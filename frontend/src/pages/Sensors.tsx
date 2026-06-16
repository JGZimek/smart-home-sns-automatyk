import { useEffect, useState } from "react";
import { type SensorData } from "../models/SensorData";
import SensorCard from "../components/SensorCard";

export default function Sensors() {
  const [sensors, setSensor] = useState<SensorData[]>([]);

  useEffect(() => {
    fetch("http://localhost:3000/readings")
      .then((res) => res.json())
      .then((json) => {
        setSensor(json);
      });
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
