#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
use std::{net::UdpSocket, sync::{Arc, Mutex, atomic::{AtomicBool, AtomicUsize, Ordering}}, thread, time::Duration};
use tauri::ipc::{Channel, Response};

struct Listener { stop: Arc<AtomicBool>, in_flight: Arc<AtomicUsize>, thread: thread::JoinHandle<()> }
#[derive(Default)]
struct UdpState(Mutex<Option<Listener>>);
impl UdpState {
    fn stop(&self) -> Result<(), String> {
        let mut guard = self.0.lock().map_err(|e| e.to_string())?;
        if let Some(listener) = guard.take() {
            listener.stop.store(true, Ordering::Release);
            listener.thread.join().map_err(|_| "UDP listener panicked".to_string())?;
        }
        Ok(())
    }
}
impl Drop for UdpState { fn drop(&mut self) { let _ = self.stop(); } }

#[tauri::command]
fn start_udp(state: tauri::State<'_, UdpState>, bind: String, on_packet: Channel<Response>) -> Result<(), String> {
    state.stop()?;
    let socket = UdpSocket::bind(&bind).map_err(|e| format!("Cannot bind UDP {bind}: {e}"))?;
    socket.set_read_timeout(Some(Duration::from_millis(50))).map_err(|e| e.to_string())?;
    let stop = Arc::new(AtomicBool::new(false));
    let in_flight = Arc::new(AtomicUsize::new(0));
    let stopped = stop.clone();
    let pending = in_flight.clone();
    let handle = thread::spawn(move || {
        // One datagram per FlatBuffer; no IP-level application fragmentation.
        let mut packet = [0u8; 65535];
        while !stopped.load(Ordering::Acquire) {
            match socket.recv_from(&mut packet) {
                Ok((size, _)) if size > 0 => {
                    // Keep at most 32 binary payloads pending in the webview IPC queue.
                    if pending.load(Ordering::Acquire) >= 32 { continue; }
                    pending.fetch_add(1, Ordering::AcqRel);
                    if on_packet.send(Response::new(packet[..size].to_vec())).is_err() { break; }
                }
                Ok(_) => {},
                Err(e) if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut | std::io::ErrorKind::Interrupted) => {},
                Err(_) => break,
            }
        }
    });
    *state.0.lock().map_err(|e| e.to_string())? = Some(Listener { stop, in_flight, thread: handle });
    Ok(())
}

#[tauri::command]
fn ack_udp(state: tauri::State<'_, UdpState>) {
    if let Ok(guard) = state.0.lock() {
        if let Some(listener) = guard.as_ref() {
            let _ = listener.in_flight.fetch_update(Ordering::AcqRel, Ordering::Acquire, |n| Some(n.saturating_sub(1)));
        }
    }
}
#[tauri::command]
fn stop_udp(state: tauri::State<'_, UdpState>) -> Result<(), String> { state.stop() }
fn main() {
    tauri::Builder::default().manage(UdpState::default())
        .invoke_handler(tauri::generate_handler![start_udp, stop_udp, ack_udp])
        .run(tauri::tauri_build_context!()).expect("Error running talOS Studio");
}
