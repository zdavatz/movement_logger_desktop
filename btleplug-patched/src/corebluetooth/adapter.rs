use super::internal::{
    run_corebluetooth_thread, CoreBluetoothEvent, CoreBluetoothMessage, CoreBluetoothReply,
    CoreBluetoothReplyFuture,
};
use super::peripheral::{Peripheral, PeripheralId};
use crate::api::{Central, CentralEvent, CentralState, ScanFilter};
use crate::common::adapter_manager::AdapterManager;
use crate::{Error, Result};
use async_trait::async_trait;
use futures::channel::mpsc::{self, Sender};
use futures::sink::SinkExt;
use futures::stream::{Stream, StreamExt};
use log::*;
use objc2_core_bluetooth::CBManagerState;
use std::pin::Pin;
use std::sync::Arc;
use tokio::task;

/// Implementation of [api::Central](crate::api::Central).
#[derive(Clone, Debug)]
pub struct Adapter {
    manager: Arc<AdapterManager<Peripheral>>,
    sender: Sender<CoreBluetoothMessage>,
}

fn get_central_state(state: CBManagerState) -> CentralState {
    match state {
        CBManagerState::PoweredOn => CentralState::PoweredOn,
        CBManagerState::PoweredOff => CentralState::PoweredOff,
        _ => CentralState::Unknown,
    }
}

impl Adapter {
    pub(crate) async fn new() -> Result<Self> {
        let (sender, mut receiver) = mpsc::channel(256);
        let adapter_sender = run_corebluetooth_thread(sender)?;
        // Since init currently blocked until the state update, we know the
        // receiver is dropped after that. We can pick it up here and make it
        // part of our event loop to update our peripherals.
        debug!("Waiting on adapter connect");
        if !matches!(
            receiver.next().await,
            Some(CoreBluetoothEvent::DidUpdateState { state: _ })
        ) {
            return Err(Error::Other(
                "Adapter failed to connect.".to_string().into(),
            ));
        }
        debug!("Adapter connected");
        let manager = Arc::new(AdapterManager::default());

        let manager_clone = manager.clone();
        let adapter_sender_clone = adapter_sender.clone();
        task::spawn(async move {
            while let Some(msg) = receiver.next().await {
                match msg {
                    CoreBluetoothEvent::DeviceDiscovered {
                        uuid,
                        name,
                        event_receiver,
                    } => {
                        manager_clone.add_peripheral(Peripheral::new(
                            uuid,
                            name,
                            Arc::downgrade(&manager_clone),
                            event_receiver,
                            adapter_sender_clone.clone(),
                        ));
                        manager_clone.emit(CentralEvent::DeviceDiscovered(uuid.into()));
                    }
                    CoreBluetoothEvent::DeviceUpdated { uuid, name } => {
                        let id = uuid.into();
                        if let Some(entry) = manager_clone.peripheral_mut(&id) {
                            entry.value().update_name(&name);
                            manager_clone.emit(CentralEvent::DeviceUpdated(id));
                        }
                    }
                    CoreBluetoothEvent::DeviceDisconnected { uuid } => {
                        manager_clone.emit(CentralEvent::DeviceDisconnected(uuid.into()));
                    }
                    CoreBluetoothEvent::DidUpdateState { state } => {
                        let central_state = get_central_state(state);
                        manager_clone.emit(CentralEvent::StateUpdate(central_state));
                    }
                }
            }
        });

        Ok(Adapter {
            manager,
            sender: adapter_sender,
        })
    }
}

#[async_trait]
impl Central for Adapter {
    type Peripheral = Peripheral;

    async fn events(&self) -> Result<Pin<Box<dyn Stream<Item = CentralEvent> + Send>>> {
        Ok(self.manager.event_stream())
    }

    async fn start_scan(&self, filter: ScanFilter) -> Result<()> {
        self.sender
            .to_owned()
            .send(CoreBluetoothMessage::StartScanning { filter })
            .await?;
        Ok(())
    }

    async fn stop_scan(&self) -> Result<()> {
        self.sender
            .to_owned()
            .send(CoreBluetoothMessage::StopScanning)
            .await?;
        Ok(())
    }

    async fn peripherals(&self) -> Result<Vec<Peripheral>> {
        Ok(self.manager.peripherals())
    }

    async fn peripheral(&self, id: &PeripheralId) -> Result<Peripheral> {
        self.manager.peripheral(id).ok_or(Error::DeviceNotFound)
    }

    /// Vendored patch: retrieve a peripheral by its (host-stable) CoreBluetooth
    /// identifier WITHOUT scanning — the macOS equivalent of iOS
    /// `retrievePeripherals(withIdentifiers:)`. Repopulates BOTH btleplug maps
    /// (Map A = CB-thread `self.peripherals`; Map B = this `AdapterManager`),
    /// which are purged on disconnect, so a subsequent `Peripheral::connect()`
    /// issues a real pending connect instead of the silent no-op a
    /// disconnect-purged id triggers today.
    async fn add_peripheral(&self, id: &PeripheralId) -> Result<Peripheral> {
        // NOTE: deliberately NO fast-path on `self.manager.peripheral(id)`
        // (Map B). On a real drop the CB thread purges Map A synchronously but
        // Map B is purged only later by this Adapter's event task, so a Map-B
        // hit does NOT imply the peripheral is connectable via Map A — issuing
        // connect() on such a stale handle no-ops and stalls the full
        // LINK_OP_TIMEOUT. Always route through RetrievePeripheral: it is
        // idempotent (on_discovered_peripheral's contains_key guard) and
        // guarantees Map A is populated whenever Map B becomes populated.
        let fut = CoreBluetoothReplyFuture::default();
        self.sender
            .to_owned()
            .send(CoreBluetoothMessage::RetrievePeripheral {
                peripheral_uuid: id.uuid(),
                future: fut.get_state_clone(),
            })
            .await?;
        match fut.await {
            CoreBluetoothReply::Ok => {}
            CoreBluetoothReply::Err(msg) => return Err(Error::RuntimeError(msg)),
            _ => return Err(Error::DeviceNotFound),
        }
        // Map B is populated from the dispatched `DeviceDiscovered` on THIS
        // Adapter's spawned event task (see `Adapter::new`), a different task
        // from this caller — briefly poll until it lands (≤1 s). This sleep
        // runs on the CALLER's runtime (the desktop BLE worker, built with
        // enable_all() → has a time driver), NOT btleplug's timerless
        // new_current_thread runtime, so it never panics "no reactor". Because
        // Map B is only ever populated via a `DeviceDiscovered` whose
        // `on_discovered_peripheral` also inserted Map A, a Map-B hit here
        // implies Map A is populated too.
        for _ in 0..50 {
            if let Some(p) = self.manager.peripheral(id) {
                return Ok(p);
            }
            tokio::time::sleep(std::time::Duration::from_millis(20)).await;
        }
        Err(Error::DeviceNotFound)
    }

    async fn adapter_info(&self) -> Result<String> {
        // TODO: Get information about the adapter.
        Ok("CoreBluetooth".to_string())
    }

    async fn adapter_state(&self) -> Result<CentralState> {
        let fut = CoreBluetoothReplyFuture::default();
        self.sender
            .to_owned()
            .send(CoreBluetoothMessage::GetAdapterState {
                future: fut.get_state_clone(),
            })
            .await?;

        match fut.await {
            CoreBluetoothReply::AdapterState(state) => {
                let central_state = get_central_state(state);
                return Ok(central_state.clone());
            }
            _ => panic!("Shouldn't get anything but a AdapterState!"),
        }
    }
}
