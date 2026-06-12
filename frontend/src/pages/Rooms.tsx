import { Typography } from "@mui/material";
import { useEffect, useState } from "react";
import { RoomCard } from "../components/RoomCard";
import { BackButton } from "../components/BackButton";

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
      <BackButton destination="/" />
      {rooms ? (
        rooms.map((room) => <RoomCard key={room.id} room={room} />)
      ) : (
        <Typography> no rooms found </Typography>
      )}
    </>
  );
};
