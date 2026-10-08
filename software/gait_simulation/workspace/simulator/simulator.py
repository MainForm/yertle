"""PyBullet simulator lifecycle and public object-management API."""

import time
from pathlib import Path

import pybullet as p
import pybullet_data

from .urdf_manager import URDFManager
from .urdf_object import URDFObject

from collections.abc import Callable
from enum import Enum

class KeyEvent(Enum):
    PRESSED = p.KEY_WAS_TRIGGERED
    HELD = p.KEY_IS_DOWN
    RELEASED = p.KEY_WAS_RELEASED

class Key(Enum):
    UP = p.B3G_UP_ARROW
    DOWN = p.B3G_DOWN_ARROW
    LEFT = p.B3G_LEFT_ARROW
    RIGHT = p.B3G_RIGHT_ARROW
    SPACE = ord(" ")
    ENTER = 13
    ESCAPE = 27

class Simulator:
    # ---------------------------------------------------------------------------
    # region Context Management

    def __init__(
        self,
        gravity: float = -9.81,
        fps: int = 240,
        use_gui: bool = True,
    ) -> None:
        if fps <= 0:
            raise ValueError("fps must be greater than 0.")

        connection_mode = p.GUI if use_gui else p.DIRECT
        self._physics_client = p.connect(connection_mode)
        if self._physics_client < 0:
            raise RuntimeError("Failed to connect to PyBullet.")

        self._fps = fps
        self._closed = False

        self._urdf_manager = URDFManager(self._physics_client)

        self._key_callbacks: dict[
            tuple[int, KeyEvent],
            Callable[[], None],
        ] = {}

        p.setAdditionalSearchPath(
            pybullet_data.getDataPath(),
            physicsClientId=self._physics_client,
        )
        p.setGravity(0, 0, gravity, physicsClientId=self._physics_client)
        p.setTimeStep(1.0 / fps, physicsClientId=self._physics_client)


    # __enter__ and __exit__ support the with statement.
    def __enter__(self) -> "Simulator":
        self._ensure_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    # endregion
    # ---------------------------------------------------------------------------


    # ---------------------------------------------------------------------------
    # region Private Methods

    def _ensure_open(self) -> None:
        if not self.is_running():
            raise RuntimeError("Simulator is already closed.")

    def _dispatch_keyboard_events(self) -> None:
        keys = p.getKeyboardEvents(
            physicsClientId=self._physics_client,
        )

        # 콜백 안에서 등록을 변경해도 순회에 영향을 주지 않도록 복사
        for (key_code, event), callback in tuple(self._key_callbacks.items()):
            if keys.get(key_code, 0) & event.value:
                callback()

    # endregion
    # ---------------------------------------------------------------------------


    # ---------------------------------------------------------------------------
    # region Public Methods
    @property
    def client_id(self) -> int:
        """Return the ID of the owned PyBullet connection."""
        self._ensure_open()
        return self._physics_client

    @property
    def fps(self) -> int:
        """Return the FPS configured for this simulation."""
        return self._fps

    def step(self) -> None:
        """Advance the physics world by one configured time step."""
        self._ensure_open()
        self._dispatch_keyboard_events()

        p.stepSimulation(physicsClientId=self._physics_client)
        time.sleep(1.0 / self._fps)

    def is_running(self) -> bool:
        """Return whether the PyBullet connection is still active."""
        return not self._closed and bool(p.isConnected(self._physics_client))

    def register_key_callback(
        self,
        key: str | Key | int,
        event: KeyEvent,
        callback: Callable[[], None],
    ) -> None:
        if isinstance(key, Key):
            key_code = key.value
        elif isinstance(key, str):
            if len(key) != 1:
                raise ValueError("문자 키는 한 글자여야 합니다.")
            key_code = ord(key)
        elif isinstance(key, int):
            key_code = key
        else:
            raise TypeError("key는 str, Key 또는 int여야 합니다.")

        self._key_callbacks[(key_code, event)] = callback


    def close(self) -> None:
        """Invalidate all objects and close the owned PyBullet world."""
        if self._closed:
            return
        
        self._urdf_manager.close()

        if p.isConnected(self._physics_client):
            p.disconnect(self._physics_client)
        self._closed = True

    # URDF Management methods

    def load_urdf(
        self,
        path: str | Path,
        base_position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        base_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        use_fixed_base: bool = False,
        global_scaling: float = 1.0,
    ) -> URDFObject:
        """Load a general URDF object and transfer ownership to Simulator."""
        self._ensure_open()
        return self._urdf_manager.load(
            path,
            base_position,
            base_orientation,
            use_fixed_base,
            global_scaling,
        )

    def remove_urdf(self, obj: URDFObject) -> None:
        """Remove one object owned by this simulation."""
        self._ensure_open()
        self._urdf_manager.remove(obj)

    # endregion
    # ---------------------------------------------------------------------------