import { Card, CardActionArea, CardMedia, Grid, Typography } from "@mui/material";
import { useNavigate } from "react-router-dom";
import { ROUTES } from "../Routes";
import sensorImg from "../assets/sensor.png"
import gateIcon from "../assets/gate.png"
import roomsIcon from "../assets/room.png"
import styled from "@emotion/styled";

const GridItem = styled(Grid)(() => ({
  display: "flex",
  justifyContent: "center",


}))

export default function MainPage() {
  const navigate = useNavigate()
  const buttons = [
    {
      image: sensorImg,
      text: "Sensors",
      nav: ROUTES.SENSORS,
    },
    {
      image: gateIcon,
      text: "Gate",
      nav: ROUTES.GATE,
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
        {buttons.map((btn, index) => (
          <GridItem size={6} key={index}>
            <Card sx={{
              padding: "2em"
            }}>
              <CardActionArea onClick={() => navigate(btn.nav)}>
                <CardMedia
                  sx={{
                    // height: "50px",
                    // width: "auto"
                  }}
                  component="img"
                  image={btn.image} />
                <Typography>{btn.text}</Typography>
              </CardActionArea>
            </Card>
          </GridItem>
        ))}
      </Grid>
    </>
  );
}