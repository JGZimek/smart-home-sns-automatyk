import { Card, CardActionArea, CardContent, styled, Typography } from "@mui/material";
import { type SensorData } from "../models/SensorData";

interface SensorProps {
  sensor: SensorData;
}

const CardWrapper = styled(Card)(({ theme }) => ({
  marginBottom: theme.spacing(1)
}))

export const SensorCard: React.FC<SensorProps> = ({ sensor }) => {
  return (
    <CardWrapper>
      <CardActionArea>
        <CardContent>
          <Typography>
            {sensor.sensor_type}: {sensor.value}
            {sensor.unit}
          </Typography>
          <Typography>Room: {sensor.room}</Typography>
        </CardContent>
      </CardActionArea>
    </CardWrapper>
  );
};
export default SensorCard;
