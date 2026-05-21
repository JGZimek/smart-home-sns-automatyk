import { Card, CardActionArea, CardMedia, Grid, Typography } from "@mui/material";
import { useNavigate } from "react-router-dom";
import { ROUTES } from "../Routes";
import sensorImg from "../assets/sensor.png"
import roomsIcon from "../assets/room.png"
import styled from "@emotion/styled";
import GateCard from "../components/GateCard";

const GridItem = styled(Grid)(() => ({
  display: "flex",
  justifyContent: "center",
}))

const CardWrapper = styled(Card)({
  padding: "2em"

})

export default function MainPage() {
  const navigate = useNavigate()
  const buttons = [
    {
      image: sensorImg,
      text: "Sensors",
      nav: ROUTES.SENSORS,
    },
    {
      image: roomsIcon,
      text: "Rooms",
      nav: ROUTES.ROOMS,
    },
  ]
  return (
    <>
      <Grid container spacing={1}>
        <Grid size={12} >
          <GateCard />
        </Grid>
        {buttons.map((btn, index) => (
          <GridItem size={6} key={index}>
            <CardWrapper>
              <CardActionArea onClick={() => navigate(btn.nav)}>
                <CardMedia
                  component="img"
                  image={btn.image} />
                <Typography>{btn.text}</Typography>
              </CardActionArea>
            </CardWrapper>
          </GridItem>
        ))}
      </Grid>
    </>
  );
}