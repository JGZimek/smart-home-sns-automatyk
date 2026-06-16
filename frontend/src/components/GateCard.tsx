import {
  Box,
  Button,
  Card,
  CardActions,
  CardContent,
  CardMedia,
  Chip,
  styled,
  Typography,
} from "@mui/material";
import gateIcon from "../assets/gate.png";
import { useEffect, useState } from "react";

interface Gate {
  isOpened: boolean;
}

const CardWrapper = styled(Card)(() => ({
  display: "flex",
  alignItems: "center",
  justifyContent: "space-between",
  gap: 16,
  padding: 16,
  marginBottom: 16,
}));

const CardImageWrapper = styled(CardMedia)(() => ({
  height: 56,
  width: 56,
  flexShrink: 0,
}));

export const GateCard = () => {
  const [gate, setGate] = useState<Gate>();

  useEffect(() => {
    fetch("http://localhost:3000/gate")
      .then((res) => res.json())
      .then((json) => {
        setGate(json);
      });
  }, []);

  function setGateStatus() {
    if (!gate) return;

    fetch("http://localhost:3000/gate", {
      method: "PATCH",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        ...gate,
        isOpened: !gate.isOpened,
      }),
    })
      .then((res) => res.json())
      .then((json) => {
        setGate(json);
      });
  }

  return (
    <CardWrapper>
      {gate ? (
        <>
          <Box sx={{ display: "flex", alignItems: "center", gap: 2 }}>
            <CardImageWrapper image={gateIcon} title="Gate image" />
            <CardContent sx={{ p: 0, "&:last-child": { pb: 0 } }}>
              <Typography variant="h3" component="h2">
                Gate
              </Typography>
              <Chip
                color={gate.isOpened ? "success" : "default"}
                label={gate.isOpened ? "Opened" : "Closed"}
                size="small"
                sx={{ mt: 1 }}
              />
            </CardContent>
          </Box>
          <CardActions sx={{ p: 0 }}>
            <Button
              onClick={setGateStatus}
              variant="contained"
              color={gate.isOpened ? "warning" : "primary"}
            >
              {gate.isOpened ? "Close gate" : "Open gate"}
            </Button>
          </CardActions>
        </>
      ) : (
        <>
          <Box sx={{ display: "flex", alignItems: "center", gap: 2 }}>
            <CardImageWrapper image={gateIcon} title="Gate image" />
            <CardContent sx={{ p: 0, "&:last-child": { pb: 0 } }}>
              <Typography variant="h3" component="h2">
                Gate
              </Typography>
              <Typography color="text.secondary">Error fetching gate data</Typography>
            </CardContent>
          </Box>
          <CardActions sx={{ p: 0 }}>
            <Button disabled variant="contained">
              Unavailable
            </Button>
          </CardActions>
        </>
      )}
    </CardWrapper>
  );
};

export default GateCard;
