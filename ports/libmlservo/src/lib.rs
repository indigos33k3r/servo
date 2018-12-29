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
use std::mem;
use std::ptr;
use keyboard_types::{Code, Key, KeyboardEvent, KeyState, Modifiers, Location};
use winit::{ElementState, KeyboardInput, ModifiersState, VirtualKeyCode};














































/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/* // Some shortcuts use Cmd on Mac and Control on other systems.
#[cfg(target_os = "macos")]
const CMD_OR_CONTROL: Modifiers = Modifiers::META;
#[cfg(not(target_os = "macos"))]
const CMD_OR_CONTROL: Modifiers = Modifiers::CONTROL;

// Some shortcuts use Cmd on Mac and Alt on other systems.
#[cfg(target_os = "macos")]
const CMD_OR_ALT: Modifiers = Modifiers::META;
#[cfg(not(target_os = "macos"))]
const CMD_OR_ALT: Modifiers = Modifiers::ALT; */

fn get_servo_key_from_winit_key(key: Option<VirtualKeyCode>) -> Key {
    use winit::VirtualKeyCode::*;
    // TODO: figure out how to map NavigateForward, NavigateBackward
    // TODO: map the remaining keys if possible
    let key = if let Some(key) = key {
        key
    } else {
        return Key::Unidentified;
    };
    match key {
        // printable: Key1 to Key0
        // printable: A to Z
        Escape => Key::Escape,
        F1 => Key::F1,
        F2 => Key::F2,
        F3 => Key::F3,
        F4 => Key::F4,
        F5 => Key::F5,
        F6 => Key::F6,
        F7 => Key::F7,
        F8 => Key::F8,
        F9 => Key::F9,
        F10 => Key::F10,
        F11 => Key::F11,
        F12 => Key::F12,
        // F13 to F15 are not mapped
        Snapshot => Key::PrintScreen,
        // Scroll not mapped
        Pause => Key::Pause,
        Insert => Key::Insert,
        Home => Key::Home,
        Delete => Key::Delete,
        End => Key::End,
        PageDown => Key::PageDown,
        PageUp => Key::PageUp,
        Left => Key::ArrowLeft,
        Up => Key::ArrowUp,
        Right => Key::ArrowRight,
        Down => Key::ArrowDown,
        Back => Key::Backspace,
        Return => Key::Enter,
        // printable: Space
        Compose => Key::Compose,
        // Caret not mapped
        Numlock => Key::NumLock,
        // printable: Numpad0 to Numpad9
        // AbntC1 and AbntC2 not mapped
        // printable: Add, Apostrophe,
        // Apps, At, Ax not mapped
        // printable: Backslash,
        Calculator => Key::LaunchApplication2,
        Capital => Key::CapsLock,
        // printable: Colon, Comma,
        Convert => Key::Convert,
        // not mapped: Decimal,
        // printable: Divide, Equals, Grave,
        Kana => Key::KanaMode,
        Kanji => Key::KanjiMode,
        LAlt => Key::Alt,
        // printable: LBracket,
        LControl => Key::Control,
        LShift => Key::Shift,
        LWin => Key::Meta,
        Mail => Key::LaunchMail,
        // not mapped: MediaSelect,
        MediaStop => Key::MediaStop,
        // printable: Minus, Multiply,
        Mute => Key::AudioVolumeMute,
        MyComputer => Key::LaunchApplication1,
        // not mapped: NavigateForward, NavigateBackward
        NextTrack => Key::MediaTrackNext,
        NoConvert => Key::NonConvert,
        // printable: NumpadComma, NumpadEnter, NumpadEquals,
        // not mapped: OEM102,
        // printable: Period,
        PlayPause => Key::MediaPlayPause,
        Power => Key::Power,
        PrevTrack => Key::MediaTrackPrevious,
        RAlt => Key::Alt,
        // printable RBracket
        RControl => Key::Control,
        RShift => Key::Shift,
        RWin => Key::Meta,
        // printable Semicolon, Slash
        Sleep => Key::Standby,
        // not mapped: Stop,
        // printable Subtract,
        // not mapped: Sysrq,
        Tab => Key::Tab,
        // printable: Underline,
        // not mapped: Unlabeled,
        VolumeDown => Key::AudioVolumeDown,
        VolumeUp => Key::AudioVolumeUp,
        Wake => Key::WakeUp,
        WebBack => Key::BrowserBack,
        WebFavorites => Key::BrowserFavorites,
        WebForward => Key::BrowserForward,
        WebHome => Key::BrowserHome,
        WebRefresh => Key::BrowserRefresh,
        WebSearch => Key::BrowserSearch,
        WebStop => Key::BrowserStop,
        // printable Yen,
        Copy => Key::Copy,
        Paste => Key::Paste,
        Cut => Key::Cut,
        _ => Key::Unidentified,
    }
}

fn get_servo_location_from_winit_key(key: Option<VirtualKeyCode>) -> Location {
    use winit::VirtualKeyCode::*;
    // TODO: add more numpad keys
    let key = if let Some(key) = key {
        key
    } else {
        return Location::Standard;
    };
    match key {
        LShift | LControl | LAlt | LWin => Location::Left,
        RShift | RControl | RAlt | RWin => Location::Right,
        Numpad0 | Numpad1 | Numpad2 | Numpad3 | Numpad4 | Numpad5 | Numpad6 | Numpad7 |
        Numpad8 | Numpad9 => Location::Numpad,
        NumpadComma | NumpadEnter | NumpadEquals => Location::Numpad,
        _ => Location::Standard,
    }
}

#[cfg(target_os = "linux")]
fn get_servo_code_from_scancode(scancode: u32) -> Code {
    // TODO: Map more codes
    use keyboard_types::Code::*;
    match scancode {
        1 => Escape,
        2 => Digit1,
        3 => Digit2,
        4 => Digit3,
        5 => Digit4,
        6 => Digit5,
        7 => Digit6,
        8 => Digit7,
        9 => Digit8,
        10 => Digit9,
        11 => Digit0,

        14 => Backspace,
        15 => Tab,
        16 => KeyQ,
        17 => KeyW,
        18 => KeyE,
        19 => KeyR,
        20 => KeyT,
        21 => KeyY,
        22 => KeyU,
        23 => KeyI,
        24 => KeyO,
        25 => KeyP,
        26 => BracketLeft,
        27 => BracketRight,
        28 => Enter,

        30 => KeyA,
        31 => KeyS,
        32 => KeyD,
        33 => KeyF,
        34 => KeyG,
        35 => KeyH,
        36 => KeyJ,
        37 => KeyK,
        38 => KeyL,
        39 => Semicolon,
        40 => Quote,

        42 => ShiftLeft,
        43 => Backslash,
        44 => KeyZ,
        45 => KeyX,
        46 => KeyC,
        47 => KeyV,
        48 => KeyB,
        49 => KeyN,
        50 => KeyM,
        51 => Comma,
        52 => Period,
        53 => Slash,
        54 => ShiftRight,

        57 => Space,

        59 => F1,
        60 => F2,
        61 => F3,
        62 => F4,
        63 => F5,
        64 => F6,
        65 => F7,
        66 => F8,
        67 => F9,
        68 => F10,

        87 => F11,
        88 => F12,

        103 => ArrowUp,
        104 => PageUp,
        105 => ArrowLeft,
        106 => ArrowRight,

        102 => Home,
        107 => End,
        108 => ArrowDown,
        109 => PageDown,
        110 => Insert,
        111 => Delete,

        _ => Unidentified,
    }
}

#[cfg(not(target_os = "linux"))]
fn get_servo_code_from_scancode(_scancode: u32) -> Code {
    // TODO: Implement for Windows and Mac OS
    Code::Unidentified
}

fn get_modifiers(mods: ModifiersState) -> Modifiers {
    let mut modifiers = Modifiers::empty();
    modifiers.set(Modifiers::CONTROL, mods.ctrl);
    modifiers.set(Modifiers::SHIFT, mods.shift);
    modifiers.set(Modifiers::ALT, mods.alt);
    modifiers.set(Modifiers::META, mods.logo);
    modifiers
}

fn keyboard_event_from_winit(input: KeyboardInput) -> KeyboardEvent {
    info!("winit keyboard input: {:?}", input);
    KeyboardEvent {
        state: match input.state {
            ElementState::Pressed => KeyState::Down,
            ElementState::Released => KeyState::Up,
        },
        key: get_servo_key_from_winit_key(input.virtual_keycode),
        code: get_servo_code_from_scancode(input.scancode),
        location: get_servo_location_from_winit_key(input.virtual_keycode),
        modifiers: get_modifiers(input.modifiers),
        repeat: false,
        is_composing: false,
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
pub unsafe extern "C" fn keyboard_servo(servo: *mut ServoInstance, scancode: u32, virtualkeycode: u32, shift: bool, ctrl: bool, alt: bool, logo: bool, down: bool) {
    if let Some(servo) = servo.as_mut() {
        let state : ElementState = match down {
          true => ElementState::Pressed,
          false => ElementState::Released,
        };
        let virtual_keycode : Option<VirtualKeyCode> = Some(mem::transmute(virtualkeycode));
        let modifiers : ModifiersState = ModifiersState{
            shift,
            ctrl,
            alt,
            logo
        };
        let keyboard_input : KeyboardInput = KeyboardInput{
          scancode,
          state,
          virtual_keycode,
          modifiers,
        };
        let keyboard_event : KeyboardEvent = keyboard_event_from_winit(keyboard_input);
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
pub unsafe extern "C" fn postmessage_servo(servo: *mut ServoInstance, data: *const u8, length: usize) {
    if let Some(servo) = servo.as_mut() {
        let mut data_vec : Vec<u8> = Vec::with_capacity(length);
        data_vec.set_len(length);
        ptr::copy(data, data_vec.as_mut_ptr(), length);

        let window_events = vec![WindowEvent::PostMessage(servo.browser_id, data_vec)];
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
        (self.boxed_logger.0)(self.boxed_app, lvl, &msg[0] as *const _ as *const _, msg.len());
    }

    fn flush(&self) {}
}
