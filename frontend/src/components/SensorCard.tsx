import { Card, CardActionArea, CardContent, Typography } from "@mui/material";
import { type SensorData } from "../models/SensorData";

interface SensorProps {
  sensor: SensorData;
}

export const SensorCard: React.FC<SensorProps> = ({ sensor }) => {
  return (
    <Card>
      <CardActionArea>
        <CardContent>
          <Typography>
            {sensor.sensor_type}: {sensor.value}
            {sensor.unit}
          </Typography>
          <Typography>Room: {sensor.room}</Typography>
        </CardContent>
      </CardActionArea>
    </Card>
  );
};
export default SensorCard;
