import {
  Card,
  CardActions,
  CardContent,
  CardHeader,
  FormControlLabel,
  FormGroup,
  FormLabel,
  Switch,
  Typography,
} from "@mui/material";
import type { Room } from "../pages/Rooms";
import type { SensorData } from "../models/SensorData";
import { useEffect, useState } from "react";
import styled from "@emotion/styled";

interface RoomProps {
  room: Room;
}

const CardWrapper = styled(Card)({
  marginBottom: 1,
});

export const RoomCard: React.FC<RoomProps> = ({ room }) => {
  const [sensors, setSensors] = useState<SensorData[]>([]);
  const [localRoom, setLocalRoom] = useState<Room>(room);

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
      <CardContent>
        {sensors
          ? sensors.map((sensor) => (
              <Typography key={sensor.id}>
                {sensor.sensor_type}: {sensor.value + sensor.unit}
              </Typography>
            ))
          : null}
      </CardContent>
      <CardActions>
        {localRoom?.lights ? (
          <>
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
          </>
        ) : null}

        {localRoom?.fan ? (
          <>
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
          </>
        ) : null}
      </CardActions>
    </CardWrapper>
  );
};
