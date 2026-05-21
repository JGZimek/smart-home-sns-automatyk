import { useEffect, useState } from "react";
import { type SensorData } from "../models/SensorData";
import SensorCard from "../components/SensorCard";
import { BackButton } from "../components/BackButton";

export default function Sensors() {
  const [sensors, setSensor] = useState<SensorData[]>([]);
  // const [hasError, setHasError] = useState(false)

  // todo: refactor after receive proper backend documentation
  useEffect(() => {
    fetch("http://localhost:3000/readings")
      .then((res) => res.json())
      .then((json) => {
        setSensor(json);
      });
  }, []);

  return (
    <>
      <BackButton destination="/" />
      {sensors.map((sensor) => (
        <SensorCard key={sensor.id} sensor={sensor}></SensorCard>
      ))}
    </>
  );
}
