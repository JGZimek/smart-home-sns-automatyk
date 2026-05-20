import { Button } from "@mui/material";
import { Link } from "react-router-dom";
import { ROUTES } from "../Routes";

export default function MainPage() {
  return (
    <>
      <h2>Pomiary</h2>
      <Button variant="contained" component={Link} to={ROUTES.SENSORS}>
        Sensors
      </Button>
    </>
  );
}
