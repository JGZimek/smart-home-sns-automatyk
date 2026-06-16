import {
  Box,
  FormControlLabel,
  FormGroup,
  FormLabel,
  Switch,
} from "@mui/material";
import type { Controllable } from "../pages/Rooms";

type DeviceType = "lights" | "fan";

interface SwitchRoomCardProps {
  title: string;
  devices: Controllable[];
  deviceType: DeviceType;
  onToggle: (deviceType: DeviceType, deviceId: number) => void;
}

export const SwitchRoomCard = ({
  title,
  devices,
  deviceType,
  onToggle,
}: SwitchRoomCardProps) => {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        alignItems: "flex-start",
        width: 170,
      }}
    >
      <FormLabel component="legend">{title}</FormLabel>
      <FormGroup>
        {devices.map((device) => (
          <FormControlLabel
            key={device.id}
            control={
              <Switch
                checked={device.isOn}
                onChange={() => onToggle(deviceType, device.id)}
              />
            }
            label={device.name}
          />
        ))}
      </FormGroup>
    </Box>
  );
};
