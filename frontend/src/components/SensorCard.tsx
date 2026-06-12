import {
  Card,
  CardActionArea,
  CardContent,
  styled,
  Typography,
} from "@mui/material";
import { useNavigate } from "react-router-dom";
import { type SensorData } from "../models/SensorData";
import { ROUTES } from "../Routes";

interface SensorProps {
  sensor: SensorData;
}

const CardWrapper = styled(Card)(({ theme }) => ({
  marginBottom: theme.spacing(1),
}));

export const SensorCard: React.FC<SensorProps> = ({ sensor }) => {
  const navigate = useNavigate();

  return (
    <CardWrapper>
      <CardActionArea
        onClick={() => navigate(ROUTES.SENSOR_HISTORY(sensor.id))}
      >
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
