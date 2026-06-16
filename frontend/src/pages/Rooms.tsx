import { Grid, Typography } from "@mui/material";
import { useEffect, useState } from "react";
import { RoomCard } from "../components/RoomCard";

export interface Room {
  id: string;
  name: string;
  lights?: Controllable[];
  fan?: Controllable[];
}

interface Controllable {
  id: number;
  name: string;
  isOn: boolean;
}

export const Rooms = () => {
  const [rooms, setRooms] = useState<Room[]>([]);

  useEffect(() => {
    fetch("http://localhost:3000/rooms")
      .then((res) => res.json())
      .then((json) => {
        setRooms(json);
      });
  }, []);

  return (
    <>
      <h1>Rooms control</h1>
      {rooms ? (
        <Grid container spacing={2}>
          {rooms.map((room) => (
            <Grid key={room.id} size={{ xs: 12, sm: 6, xl: 4}}>
              <RoomCard room={room} />
            </Grid>
          ))}
        </Grid>
      ) : (
        <Typography> no rooms found </Typography>
      )}
    </>
  );
};
