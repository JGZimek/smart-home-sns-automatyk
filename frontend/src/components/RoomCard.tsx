import {
  Box,
  Card,
  CardActions,
  CardContent,
  CardHeader,
  Link,
  Typography,
} from "@mui/material";
import type { Room } from "../pages/Rooms";
import type { SensorData } from "../models/SensorData";
import { useEffect, useState } from "react";
import styled from "@emotion/styled";
import { useNavigate } from "react-router-dom";
import { ROUTES } from "../Routes";
import { SwitchRoomCard } from "./SwitchRoomCard";
import { apiClient } from "../api/client";

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
    apiClient
      .get<SensorData[]>(`/readings?room=${room.id}`)
      .then((response) => setSensors(response.data));
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

    apiClient
      .patch(`/rooms/${localRoom.id}`, {
        [deviceType]: updatedRoom[deviceType],
      })
      .then((response) => {
        console.log("Zaktualizowano w bazie:", response.data);
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
            <SwitchRoomCard
              title="Lights"
              devices={localRoom.lights}
              deviceType="lights"
              onToggle={handleToggle}
            />
          ) : null}

          {localRoom?.fan ? (
            <SwitchRoomCard
              title="Fans"
              devices={localRoom.fan}
              deviceType="fan"
              onToggle={handleToggle}
            />
          ) : null}
        </CardActions>
      </Box>
    </CardWrapper>
  );
};
