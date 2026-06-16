import { useEffect, useState } from "react";
import { type SensorData } from "../models/SensorData";
import SensorCard from "../components/SensorCard";
import { apiClient } from "../api/client";

export default function Sensors() {
  const [sensors, setSensor] = useState<SensorData[]>([]);

  useEffect(() => {
    apiClient
      .get<SensorData[]>(`/readings`)
      .then((response) => setSensor(response.data));
  }, []);

  return (
    <>
      <h1>Sensors info</h1>
      {sensors.map((sensor) => (
        <SensorCard key={sensor.id} sensor={sensor}></SensorCard>
      ))}
    </>
  );
}
