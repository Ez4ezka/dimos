from typing import Any

class _MavlinkMessage:
    base_mode: int
    custom_mode: int
    command: int
    result: int
    def get_type(self) -> str: ...
    def get_srcSystem(self) -> int: ...
    def get_srcComponent(self) -> int: ...
    def to_dict(self) -> dict[str, Any]: ...
    def __getattr__(self, name: str) -> Any: ...

class _MavSender:
    def command_long_send(self, *args: Any, **kwargs: Any) -> None: ...
    def command_int_send(self, *args: Any, **kwargs: Any) -> None: ...
    def heartbeat_send(self, *args: Any, **kwargs: Any) -> None: ...
    def set_mode_send(self, *args: Any, **kwargs: Any) -> None: ...
    def set_position_target_local_ned_send(self, *args: Any, **kwargs: Any) -> None: ...
    def gimbal_manager_set_pitchyaw_send(self, *args: Any, **kwargs: Any) -> None: ...

class MavlinkConnection:
    target_system: int
    target_component: int
    mav: _MavSender
    def wait_heartbeat(self, timeout: float | None = ...) -> _MavlinkMessage: ...
    def recv_match(
        self,
        type: str | list[str] | None = ...,
        blocking: bool = ...,
        timeout: float | None = ...,
    ) -> _MavlinkMessage | None: ...
    def close(self) -> None: ...

def mavlink_connection(
    device: str,
    baud: int = ...,
    source_system: int = ...,
    source_component: int = ...,
    autoreconnect: bool = ...,
    **kwargs: Any,
) -> MavlinkConnection: ...

class _MavlinkConstants:
    MAV_CMD_COMPONENT_ARM_DISARM: int
    MAV_CMD_DO_SET_MODE: int
    MAV_CMD_NAV_LAND: int
    MAV_CMD_NAV_TAKEOFF: int
    MAV_CMD_SET_MESSAGE_INTERVAL: int
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW: int
    MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE: int
    MAV_FRAME_BODY_NED: int
    MAV_FRAME_LOCAL_NED: int
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: int
    MAV_MODE_FLAG_SAFETY_ARMED: int
    MAV_RESULT_ACCEPTED: int
    MAV_RESULT_TEMPORARILY_REJECTED: int
    MAV_RESULT_DENIED: int
    MAV_RESULT_FAILED: int
    MAV_TYPE_ONBOARD_CONTROLLER: int
    MAV_AUTOPILOT_INVALID: int
    MAV_STATE_ACTIVE: int
    MAV_LANDED_STATE_UNDEFINED: int
    MAV_LANDED_STATE_ON_GROUND: int
    MAV_LANDED_STATE_IN_AIR: int
    MAV_LANDED_STATE_TAKEOFF: int
    MAV_LANDED_STATE_LANDING: int
    GIMBAL_MANAGER_FLAGS_RETRACT: int
    GIMBAL_MANAGER_FLAGS_NEUTRAL: int
    GIMBAL_MANAGER_FLAGS_ROLL_LOCK: int
    GIMBAL_MANAGER_FLAGS_PITCH_LOCK: int
    GIMBAL_MANAGER_FLAGS_YAW_LOCK: int
    GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME: int
    GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME: int

mavlink: _MavlinkConstants
