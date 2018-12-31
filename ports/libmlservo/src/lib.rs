/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

use egl::egl::EGLContext;
use egl::egl::EGLDisplay;
use egl::egl::EGLSurface;
use egl::egl::MakeCurrent;
use egl::egl::SwapBuffers;
use egl::eglext::eglGetProcAddress;
use log::info;
use log::warn;
use servo::compositing::windowing::AnimationState;
use servo::compositing::windowing::EmbedderCoordinates;
use servo::compositing::windowing::MouseWindowEvent;
use servo::compositing::windowing::WindowEvent;
use servo::compositing::windowing::WindowMethods;
use servo::embedder_traits::resources::Resource;
use servo::embedder_traits::resources::ResourceReaderMethods;
use servo::embedder_traits::EmbedderMsg;
use servo::embedder_traits::EventLoopWaker;
use servo::euclid::TypedPoint2D;
use servo::euclid::TypedRect;
use servo::euclid::TypedScale;
use servo::euclid::TypedSize2D;
use servo::gl;
use servo::gl::Gl;
use servo::gl::GlesFns;
use servo::msg::constellation_msg::TraversalDirection;
use servo::script_traits::MouseButton;
use servo::script_traits::TouchEventType;
use servo::servo_url::ServoUrl;
use servo::webrender_api::DevicePixel;
use servo::webrender_api::DevicePoint;
use servo::webrender_api::LayoutPixel;
use servo::webrender_api::ScrollLocation;
use servo::BrowserId;
use servo::Servo;
use smallvec::SmallVec;
use std::ffi::CStr;
use std::ffi::CString;
use std::io::Write;
use std::os::raw::c_char;
use std::os::raw::c_void;
use std::path::PathBuf;
use std::rc::Rc;
use std::thread;
use std::time::Duration;
use std::time::Instant;
use std::ptr;
use keyboard_types::{Code, Key, KeyboardEvent, KeyState, Modifiers, Location};

fn get_servo_code_from_keycode(keycode: u32) -> Code {
    use keyboard_types::Code::*;

    match keycode {
        48 => Digit0,
        49 => Digit1,
        50 => Digit2,
        51 => Digit3,
        52 => Digit4,
        53 => Digit5,
        54 => Digit6,
        55 => Digit7,
        56 => Digit8,
        57 => Digit9,
        65 => KeyA,
        66 => KeyB,
        67 => KeyC,
        68 => KeyD,
        69 => KeyE,
        70 => KeyF,
        71 => KeyG,
        72 => KeyH,
        73 => KeyI,
        74 => KeyJ,
        75 => KeyK,
        76 => KeyL,
        77 => KeyM,
        78 => KeyN,
        79 => KeyO,
        80 => KeyP,
        81 => KeyQ,
        82 => KeyR,
        83 => KeyS,
        84 => KeyT,
        85 => KeyU,
        86 => KeyV,
        87 => KeyW,
        88 => KeyX,
        89 => KeyY,
        90 => KeyZ,
        27 => Escape,
        112 => F1,
        113 => F2,
        114 => F3,
        115 => F4,
        116 => F5,
        117 => F6,
        118 => F7,
        119 => F8,
        120 => F9,
        121 => F10,
        122 => F11,
        123 => F12,
        37 => ArrowLeft,
        38 => ArrowUp,
        39 => ArrowRight,
        40 => ArrowDown,
        13 => Enter,
        32 => Space,
        9 => Tab,
        8 => Backspace,
        20 => CapsLock,
        192 => Backquote,
        189 => Minus,
        187 => Equal,
        219 => BracketLeft,
        221 => BracketRight,
        220 => Backslash,
        186 => Semicolon,
        222 => Quote,
        188 => Comma,
        190 => Period,
        191 => Slash,
        16 => ShiftLeft,
        17 => ControlLeft,
        18 => AltLeft,
        91 => MetaLeft,
        _ => Unidentified,
    }
}

fn get_servo_key_from_keycode(keycode: u32) -> Key {
    // use winit::VirtualKeyCode::*;

    match keycode {
        /* 48 => Digit0,
        49 => Digit1,
        50 => Digit2,
        51 => Digit3,
        52 => Digit4,
        53 => Digit5,
        54 => Digit6,
        55 => Digit7,
        56 => Digit8,
        57 => Digit9,
        65 => KeyA,
        66 => KeyB,
        67 => KeyC,
        68 => KeyD,
        69 => KeyE,
        70 => KeyF,
        71 => KeyG,
        72 => KeyH,
        73 => KeyI,
        74 => KeyJ,
        75 => KeyK,
        76 => KeyL,
        77 => KeyM,
        78 => KeyN,
        79 => KeyO,
        80 => KeyP,
        81 => KeyQ,
        82 => KeyR,
        83 => KeyS,
        84 => KeyT,
        85 => KeyU,
        86 => KeyV,
        87 => KeyW,
        88 => KeyX,
        89 => KeyY,
        90 => KeyZ, */
        27 => Key::Escape,
        112 => Key::F1,
        113 => Key::F2,
        114 => Key::F3,
        115 => Key::F4,
        116 => Key::F5,
        117 => Key::F6,
        118 => Key::F7,
        119 => Key::F8,
        120 => Key::F9,
        121 => Key::F10,
        122 => Key::F11,
        123 => Key::F12,
        37 => Key::ArrowLeft,
        38 => Key::ArrowUp,
        39 => Key::ArrowRight,
        40 => Key::ArrowDown,
        13 => Key::Enter,
        // 32 => Space,
        9 => Key::Tab,
        8 => Key::Backspace,
        20 => Key::CapsLock,
        // 192 => Grave,
        // 189 => Subtract,
        // 187 => Equals,
        // 219 => BracketLeft,
        // 221 => BracketRight,
        // 220 => Backslash,
        // 186 => Semicolon,
        // 222 => Apostrophe,
        // 188 => Comma,
        // 190 => Period,
        // 191 => Slash,
        16 => Key::Shift,
        17 => Key::Control,
        18 => Key::Alt,
        91 => Key::Meta,
        _ => Key::Unidentified,
    }
}

#[repr(u32)]
pub enum MLLogLevel {
    Fatal = 0,
    Error = 1,
    Warning = 2,
    Info = 3,
    Debug = 4,
    Verbose = 5,
}

#[repr(transparent)]
pub struct MLLogger(extern "C" fn(MLApp, MLLogLevel, *const c_char, usize));

#[repr(transparent)]
pub struct MLHistoryUpdate(extern "C" fn(MLApp, bool, *const c_char, bool));

#[repr(transparent)]
pub struct MLPresentUpdate(extern "C" fn(MLApp));

#[repr(transparent)]
#[derive(Clone, Copy)]
pub struct MLApp(*mut c_void);

const LOG_LEVEL: log::LevelFilter = log::LevelFilter::Info;

#[no_mangle]
pub unsafe extern "C" fn init_servo(
    ctxt: EGLContext,
    surf: EGLSurface,
    disp: EGLDisplay,
    app: MLApp,
    logger: MLLogger,
    history_update: MLHistoryUpdate,
    present_update: MLPresentUpdate,
    url: *const c_char,
    width: u32,
    height: u32,
    hidpi: f32,
) -> *mut ServoInstance {
    // Servo initialization goes here!
    servo::embedder_traits::resources::set(Box::new(ResourceReaderInstance::new()));
    let logger_wrap = MLLoggerWrap{
      boxed_app: app,
      boxed_logger: logger,
    };
    let _ = log::set_boxed_logger(Box::new(logger_wrap));
    log::set_max_level(LOG_LEVEL);
    let gl = GlesFns::load_with(|symbol| {
        let cstr = CString::new(symbol).expect("Failed to convert GL symbol to a char*");
        eglGetProcAddress(cstr.as_ptr() as _) as _
    });

    info!("OpenGL version {}", gl.get_string(gl::VERSION));
    let window = Rc::new(WindowInstance {
        ctxt: ctxt,
        surf: surf,
        disp: disp,
        gl: gl,
        width: width,
        height: height,
        hidpi: hidpi,
        app: app,
        present_update: present_update,
    });

    info!("Starting servo");
    let mut servo = Servo::new(window);
    let browser_id = BrowserId::new();

    let blank_url = ServoUrl::parse("about:blank").expect("Failed to parse about:blank!");
    let url = CStr::from_ptr(url).to_str().unwrap_or("about:blank");
    let url = ServoUrl::parse(url).unwrap_or(blank_url);
    servo.handle_events(vec![WindowEvent::NewBrowser(url, browser_id)]);

    let result = Box::new(ServoInstance {
        app: app,
        browser_id: browser_id,
        history_update: history_update,
        scroll_state: ScrollState::TriggerUp,
        scroll_scale: TypedScale::new(SCROLL_SCALE / hidpi),
        servo: servo,
    });
    Box::into_raw(result)
}

#[no_mangle]
pub unsafe extern "C" fn heartbeat_servo(servo: *mut ServoInstance) {
    // Servo heartbeat goes here!
    if let Some(servo) = servo.as_mut() {
        servo.servo.handle_events(vec![]);
        for ((_browser_id, event)) in servo.servo.get_events() {
            match event {
                // Respond to any messages with a response channel
                // to avoid deadlocking the constellation
                EmbedderMsg::AllowNavigation(_url, sender) => {
                    let _ = sender.send(true);
                },
                EmbedderMsg::GetSelectedBluetoothDevice(_, sender) => {
                    let _ = sender.send(None);
                },
                EmbedderMsg::AllowUnload(sender) => {
                    let _ = sender.send(true);
                },
                EmbedderMsg::Alert(_, sender) => {
                    let _ = sender.send(());
                },
                EmbedderMsg::AllowOpeningBrowser(sender) => {
                    let _ = sender.send(false);
                },
                // Update the history UI
                EmbedderMsg::HistoryChanged(urls, index) => {
                    if let Some(url) = urls.get(index) {
                        if let Ok(cstr) = CString::new(url.as_str()) {
                            let can_go_back = index > 0;
                            let can_go_fwd = (index + 1) < urls.len();
                            (servo.history_update.0)(
                                servo.app,
                                can_go_back,
                                cstr.as_ptr(),
                                can_go_fwd,
                            );
                        }
                    }
                },
                // Ignore most messages for now
                EmbedderMsg::ChangePageTitle(..) |
                EmbedderMsg::BrowserCreated(..) |
                EmbedderMsg::LoadStart |
                EmbedderMsg::LoadComplete |
                EmbedderMsg::CloseBrowser |
                EmbedderMsg::Status(..) |
                EmbedderMsg::SelectFiles(..) |
                EmbedderMsg::MoveTo(..) |
                EmbedderMsg::ResizeTo(..) |
                EmbedderMsg::Keyboard(..) |
                EmbedderMsg::SetCursor(..) |
                EmbedderMsg::NewFavicon(..) |
                EmbedderMsg::HeadParsed |
                EmbedderMsg::SetFullscreenState(..) |
                EmbedderMsg::ShowIME(..) |
                EmbedderMsg::HideIME |
                EmbedderMsg::Shutdown |
                EmbedderMsg::Panic(..) => {},
            }
        }
    }
}

// Some magic numbers.

// How far does the cursor have to move for it to count as a drag rather than a click?
// (In device pixels squared, to avoid taking a sqrt when calculating move distance.)
const DRAG_CUTOFF_SQUARED: f32 = 900.0;

// How much should we scale scrolling by?
const SCROLL_SCALE: f32 = 3.0;

#[no_mangle]
pub unsafe extern "C" fn move_servo(servo: *mut ServoInstance, x: f32, y: f32) {
    // Servo's cursor was moved
    if let Some(servo) = servo.as_mut() {
        let point = DevicePoint::new(x, y);
        let (new_state, window_events) = match servo.scroll_state {
            ScrollState::TriggerUp => (
                ScrollState::TriggerUp,
                vec![WindowEvent::MouseWindowMoveEventClass(point)],
            ),
            ScrollState::TriggerDown(start)
                if (start - point).square_length() < DRAG_CUTOFF_SQUARED =>
            {
                return
            },
            ScrollState::TriggerDown(start) => (
                ScrollState::TriggerDragging(start, point),
                vec![
                    WindowEvent::MouseWindowMoveEventClass(point),
                    WindowEvent::Scroll(
                        ScrollLocation::Delta((point - start) * servo.scroll_scale),
                        start.to_i32(),
                        TouchEventType::Down,
                    ),
                ],
            ),
            ScrollState::TriggerDragging(start, prev) => (
                ScrollState::TriggerDragging(start, point),
                vec![
                    WindowEvent::MouseWindowMoveEventClass(point),
                    WindowEvent::Scroll(
                        ScrollLocation::Delta((point - prev) * servo.scroll_scale),
                        start.to_i32(),
                        TouchEventType::Move,
                    ),
                ],
            ),
        };
        servo.scroll_state = new_state;
        servo.servo.handle_events(window_events);
    }
}

#[no_mangle]
pub unsafe extern "C" fn trigger_servo(servo: *mut ServoInstance, x: f32, y: f32, down: bool) {
    // Servo was triggered
    if let Some(servo) = servo.as_mut() {
        let point = DevicePoint::new(x, y);
        let (new_state, window_events) = match servo.scroll_state {
            ScrollState::TriggerUp if down => (
                ScrollState::TriggerDown(point),
                vec![WindowEvent::MouseWindowEventClass(
                    MouseWindowEvent::MouseDown(MouseButton::Left, point),
                )],
            ),
            ScrollState::TriggerDown(start) if !down => (
                ScrollState::TriggerUp,
                vec![
                    WindowEvent::MouseWindowEventClass(MouseWindowEvent::MouseUp(
                        MouseButton::Left,
                        start,
                    )),
                    WindowEvent::MouseWindowEventClass(MouseWindowEvent::Click(
                        MouseButton::Left,
                        start,
                    )),
                    WindowEvent::MouseWindowMoveEventClass(point),
                ],
            ),
            ScrollState::TriggerDragging(start, prev) if !down => (
                ScrollState::TriggerUp,
                vec![
                    WindowEvent::Scroll(
                        ScrollLocation::Delta((point - prev) * servo.scroll_scale),
                        start.to_i32(),
                        TouchEventType::Up,
                    ),
                    WindowEvent::MouseWindowEventClass(MouseWindowEvent::MouseUp(
                        MouseButton::Left,
                        point,
                    )),
                    WindowEvent::MouseWindowMoveEventClass(point),
                ],
            ),
            _ => return,
        };
        servo.scroll_state = new_state;
        servo.servo.handle_events(window_events);
    }
}

#[no_mangle]
pub unsafe extern "C" fn keyboard_servo(servo: *mut ServoInstance, keycode: u32, shift: bool, ctrl: bool, alt: bool, logo: bool, down: bool) {
    if let Some(servo) = servo.as_mut() {
        let mut modifiers = Modifiers::empty();
        modifiers.set(Modifiers::CONTROL, ctrl);
        modifiers.set(Modifiers::SHIFT, shift);
        modifiers.set(Modifiers::ALT, alt);
        modifiers.set(Modifiers::META, logo);

        let keyboard_event : KeyboardEvent = KeyboardEvent{
            state: match down {
                true => KeyState::Down,
                false => KeyState::Up,
            },
            key: get_servo_key_from_keycode(keycode),
            code: get_servo_code_from_keycode(keycode),
            location: Location::Standard,
            modifiers,
            repeat: false,
            is_composing: false,
        };
        let window_events = vec![WindowEvent::Keyboard(keyboard_event)];
        servo.servo.handle_events(window_events);
    }
}

#[no_mangle]
pub unsafe extern "C" fn executejs_servo(servo: *mut ServoInstance, data: *const u8, length: usize) {
    if let Some(servo) = servo.as_mut() {
        let mut data_vec : Vec<u8> = Vec::with_capacity(length);
        data_vec.set_len(length);
        ptr::copy(data, data_vec.as_mut_ptr(), length);

        let script_string : String = String::from_utf8(data_vec).unwrap();

        let window_events = vec![WindowEvent::WebDriverCommand(servo.browser_id, script_string)];
        servo.servo.handle_events(window_events);
    }
}

#[no_mangle]
pub unsafe extern "C" fn traverse_servo(servo: *mut ServoInstance, delta: i32) {
    // Traverse the session history
    if let Some(servo) = servo.as_mut() {
        let window_event = if delta == 0 {
            WindowEvent::Reload(servo.browser_id)
        } else if delta < 0 {
            WindowEvent::Navigation(servo.browser_id, TraversalDirection::Back(-delta as usize))
        } else {
            WindowEvent::Navigation(
                servo.browser_id,
                TraversalDirection::Forward(delta as usize),
            )
        };
        servo.servo.handle_events(vec![window_event]);
    }
}

#[no_mangle]
pub unsafe extern "C" fn navigate_servo(servo: *mut ServoInstance, text: *const c_char) {
    if let Some(servo) = servo.as_mut() {
        let text = CStr::from_ptr(text)
            .to_str()
            .expect("Failed to convert text to UTF-8");
        let url = ServoUrl::parse(text).unwrap_or_else(|_| {
            let mut search = ServoUrl::parse("https://duckduckgo.com")
                .expect("Failed to parse search URL")
                .into_url();
            search.query_pairs_mut().append_pair("q", text);
            ServoUrl::from_url(search)
        });
        let window_event = WindowEvent::LoadUrl(servo.browser_id, url);
        servo.servo.handle_events(vec![window_event]);
    }
}

// Some magic numbers for shutdown
const SHUTDOWN_DURATION: Duration = Duration::from_secs(10);
const SHUTDOWN_POLL_INTERVAL: Duration = Duration::from_millis(100);

#[no_mangle]
pub unsafe extern "C" fn discard_servo(servo: *mut ServoInstance) {
    if let Some(servo) = servo.as_mut() {
        let mut servo = Box::from_raw(servo);
        let finish = Instant::now() + SHUTDOWN_DURATION;
        servo.servo.handle_events(vec![WindowEvent::Quit]);
        'outer: loop {
            for (_, msg) in servo.servo.get_events() {
                if let EmbedderMsg::Shutdown = msg {
                    break 'outer;
                }
            }
            if Instant::now() > finish {
                warn!("Incomplete shutdown.");
                break 'outer;
            }
            thread::sleep(SHUTDOWN_POLL_INTERVAL);
            servo.servo.handle_events(vec![]);
        }
        servo.servo.deinit();
    }
}

pub struct ServoInstance {
    app: MLApp,
    browser_id: BrowserId,
    history_update: MLHistoryUpdate,
    servo: Servo<WindowInstance>,
    scroll_state: ScrollState,
    scroll_scale: TypedScale<f32, DevicePixel, LayoutPixel>,
}

struct WindowInstance {
    ctxt: EGLContext,
    surf: EGLSurface,
    disp: EGLDisplay,
    gl: Rc<Gl>,
    width: u32,
    height: u32,
    hidpi: f32,
    app: MLApp,
    present_update: MLPresentUpdate,
}

#[derive(Clone, Copy)]
enum ScrollState {
    TriggerUp,
    TriggerDown(DevicePoint),
    TriggerDragging(DevicePoint, DevicePoint),
}

impl WindowMethods for WindowInstance {
    fn present(&self) {
        SwapBuffers(self.disp, self.surf);
        (self.present_update.0)(self.app);
    }

    fn prepare_for_composite(&self) -> bool {
        MakeCurrent(self.disp, self.surf, self.surf, self.ctxt);
        true
    }

    fn gl(&self) -> Rc<Gl> {
        self.gl.clone()
    }

    fn create_event_loop_waker(&self) -> Box<EventLoopWaker> {
        Box::new(EventLoopWakerInstance::new())
    }

    fn get_coordinates(&self) -> EmbedderCoordinates {
        EmbedderCoordinates {
            hidpi_factor: TypedScale::new(self.hidpi),
            screen: TypedSize2D::new(self.width as i32, self.height as i32),
            screen_avail: TypedSize2D::new(self.width as i32, self.height as i32),
            window: (
                TypedSize2D::new(self.width as i32, self.height as i32),
                TypedPoint2D::new(0, 0),
            ),
            framebuffer: TypedSize2D::new(self.width as i32, self.height as i32),
            viewport: TypedRect::new(
                TypedPoint2D::new(0, 0),
                TypedSize2D::new(self.width as i32, self.height as i32),
            ),
        }
    }

    fn set_animation_state(&self, _state: AnimationState) {}
}

struct EventLoopWakerInstance;

impl EventLoopWakerInstance {
    fn new() -> EventLoopWakerInstance {
        EventLoopWakerInstance
    }
}

impl EventLoopWaker for EventLoopWakerInstance {
    fn clone(&self) -> Box<EventLoopWaker + Send> {
        Box::new(EventLoopWakerInstance)
    }

    fn wake(&self) {}
}

struct ResourceReaderInstance;

impl ResourceReaderInstance {
    fn new() -> ResourceReaderInstance {
        ResourceReaderInstance
    }
}

impl ResourceReaderMethods for ResourceReaderInstance {
    fn read(&self, res: Resource) -> Vec<u8> {
        Vec::from(match res {
            Resource::Preferences => &include_bytes!("../../../resources/prefs.json")[..],
            Resource::HstsPreloadList => {
                &include_bytes!("../../../resources/hsts_preload.json")[..]
            },
            Resource::SSLCertificates => &include_bytes!("../../../resources/certs")[..],
            Resource::BadCertHTML => &include_bytes!("../../../resources/badcert.html")[..],
            Resource::NetErrorHTML => &include_bytes!("../../../resources/neterror.html")[..],
            Resource::UserAgentCSS => &include_bytes!("../../../resources/user-agent.css")[..],
            Resource::ServoCSS => &include_bytes!("../../../resources/servo.css")[..],
            Resource::PresentationalHintsCSS => {
                &include_bytes!("../../../resources/presentational-hints.css")[..]
            },
            Resource::QuirksModeCSS => &include_bytes!("../../../resources/quirks-mode.css")[..],
            Resource::RippyPNG => &include_bytes!("../../../resources/rippy.png")[..],
            Resource::DomainList => &include_bytes!("../../../resources/public_domains.txt")[..],
            Resource::BluetoothBlocklist => {
                &include_bytes!("../../../resources/gatt_blocklist.txt")[..]
            },
        })
    }

    fn sandbox_access_files(&self) -> Vec<PathBuf> {
        vec![]
    }

    fn sandbox_access_files_dirs(&self) -> Vec<PathBuf> {
        vec![]
    }
}

struct MLLoggerWrap {
  boxed_app: MLApp,
  boxed_logger: MLLogger,
}

unsafe impl Send for MLLoggerWrap {}
unsafe impl Sync for MLLoggerWrap {}

impl log::Log for MLLoggerWrap {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= LOG_LEVEL
    }

    fn log(&self, record: &log::Record) {
        let lvl = match record.level() {
            log::Level::Error => MLLogLevel::Error,
            log::Level::Warn => MLLogLevel::Warning,
            log::Level::Info => MLLogLevel::Info,
            log::Level::Debug => MLLogLevel::Debug,
            log::Level::Trace => MLLogLevel::Verbose,
        };
        let mut msg = SmallVec::<[u8; 128]>::new();
        write!(msg, "{}\0", record.args());
        (self.boxed_logger.0)(self.boxed_app, lvl, &msg[0] as *const _ as *const _, msg.len() - 1);
    }

    fn flush(&self) {}
}
