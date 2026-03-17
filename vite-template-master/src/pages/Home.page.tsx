import { SensorCard } from '@/components/SensorCard';
import { ColorSchemeToggle } from '../components/ColorSchemeToggle/ColorSchemeToggle';
import { Welcome } from '../components/Welcome/Welcome';
import { CELCIUS_DEGREES, PERCENTS } from '@/Constants';

export function HomePage() {
  return (
    <>
      {/* <Welcome /> */}
      {/* <ColorSchemeToggle /> */}
      <SensorCard quantity='temperatura' value={15} unit={CELCIUS_DEGREES} />
      <SensorCard quantity='wilgotność powietrza' value={15} unit={PERCENTS} />
    
    </>
  );
}
