import {
  Box,
  Card,
  CardActions,
  CardContent,
  CardHeader,
  FormControlLabel,
  FormGroup,
  FormLabel,
  Link,
  Switch,
  Typography,
} from "@mui/material";
import type { Room } from "../pages/Rooms";
import type { SensorData } from "../models/SensorData";
import { useEffect, useState } from "react";
import styled from "@emotion/styled";
import { useNavigate } from "react-router-dom";
import { ROUTES } from "../Routes";

interface RoomProps {
  room: Room;
}

const CardWrapper = styled(Card)({
  marginBottom: 1,
  height: "100%",
});

export const RoomCard: React.FC<RoomProps> = ({ room }) => {
  const [sensors, setSensors] = useState<SensorData[]>([]);
  const [localRoom, setLocalRoom] = useState<Room>(room);
  const navigate = useNavigate();

  useEffect(() => {
    fetch(`http://localhost:3000/readings?room=${room.id}`)
      .then((res) => res.json())
      .then((json) => {
        setSensors(json);
      });
  }, [room.id]);

  const handleToggle = (deviceType: "lights" | "fan", deviceId: number) => {
    const updatedRoom = { ...localRoom };
    const backupRoom = { ...localRoom };

    if (deviceType === "lights" && updatedRoom.lights) {
      updatedRoom.lights = updatedRoom.lights.map((light) =>
        light.id === deviceId ? { ...light, isOn: !light.isOn } : light,
      );
    } else if (deviceType === "fan" && updatedRoom.fan) {
      updatedRoom.fan = updatedRoom.fan.map((fan) =>
        fan.id === deviceId ? { ...fan, isOn: !fan.isOn } : fan,
      );
    }

    setLocalRoom(updatedRoom);

    fetch(`http://localhost:3000/rooms/${localRoom.id}`, {
      method: "PATCH",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        // Wysyłamy zaktualizowaną tablicę zależącą od tego, co kliknęliśmy
        [deviceType]: updatedRoom[deviceType],
      }),
    })
      .then((res) => res.json())
      .then((data) => {
        console.log("Zaktualizowano w bazie:", data);
      })
      .catch((err) => {
        console.error("Error updating database", err);
        setLocalRoom(backupRoom);
      });
  };

  return (
    <CardWrapper>
      <CardHeader title={room.name} />
      <Box
        sx={{
          display: "flex",
          gap: 3,
          justifyContent: "space-between",
          alignItems: "flex-start",
        }}
      >
        <CardContent sx={{ flex: 1 }}>
          {sensors
            ? sensors.map((sensor) => (
                <Typography key={sensor.id}>
                  <Link
                    component="button"
                    underline="none"
                    onClick={() => navigate(ROUTES.SENSOR_HISTORY(sensor.id))}
                    sx={{
                      color: "text.primary",
                      fontWeight: 400,
                      textAlign: "left",
                      cursor: "pointer",
                      "&:hover": {
                        color: "text.secondary",
                      },
                    }}
                  >
                    {sensor.sensor_type}: {sensor.value + sensor.unit}
                  </Link>
                </Typography>
              ))
            : null}
        </CardContent>

        <CardActions sx={{ alignItems: "flex-start", gap: 3, pr: 2, pt: 1 }}>
          {localRoom?.lights ? (
            <Box
              sx={{
                display: "flex",
                flexDirection: "column",
                alignItems: "flex-start",
              }}
            >
              <FormLabel component="legend">Lights</FormLabel>
              <FormGroup>
                {localRoom.lights.map((light) => (
                  <FormControlLabel
                    key={light.id}
                    control={
                      <Switch
                        checked={light.isOn}
                        onChange={() => handleToggle("lights", light.id)}
                      />
                    }
                    label={light.name}
                  />
                ))}
              </FormGroup>
            </Box>
          ) : null}

          {localRoom?.fan ? (
            <Box
              sx={{
                display: "flex",
                flexDirection: "column",
                alignItems: "flex-start",
              }}
            >
              <FormLabel component="legend">Fans</FormLabel>
              <FormGroup>
                {localRoom.fan.map((fan) => (
                  <FormControlLabel
                    key={fan.id}
                    control={
                      <Switch
                        checked={fan.isOn}
                        onChange={() => handleToggle("fan", fan.id)}
                      />
                    }
                    label={fan.name}
                  />
                ))}
              </FormGroup>
            </Box>
          ) : null}
        </CardActions>
      </Box>
    </CardWrapper>
  );
};
