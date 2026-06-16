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
import { apiClient } from "../api/client";

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
    apiClient
      .get<Gate>("/gate")
      .then((response) => setGate(response.data));
  }, []);

  function setGateStatus() {
    if (!gate) return;

    apiClient
      .patch<Gate>("/gate", {
        ...gate,
        isOpened: !gate.isOpened,
      })
      .then((response) => setGate(response.data));
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
