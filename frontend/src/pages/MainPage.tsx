import { GateCard } from "../components/GateCard";
import { Rooms } from "./Rooms";
import Sensors from "./Sensors";


export default function MainPage() {
  return (
    <>
      <GateCard />
      <Sensors />
      <Rooms />
    </>
  );
}
