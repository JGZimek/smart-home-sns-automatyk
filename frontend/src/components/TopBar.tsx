import { Box, AppBar, IconButton, Typography, Toolbar } from "@mui/material";
import logo from "../assets/logo1.png";
import { useNavigate } from "react-router-dom";
import styled from "@emotion/styled";

const StyledAppBar = styled(AppBar)({
  backgroundColor: "lightgray",
  color: "black",
});

const StyledToolbar = styled(Toolbar)({
  display: "flex",
  gap: "15px",
  padding: 0,
});

const TextWrapper = styled(Box)({
  display: "flex",
  justifyContent: "center",
  width: "100%",
});

export const TopBar = () => {
  const navigate = useNavigate();
  return (
    <StyledAppBar position="sticky">
      <StyledToolbar>
        <IconButton onClick={() => navigate("/")}>
          <img
            src={logo}
            alt="Science club logo"
            style={{
              height: "30px",
              width: "auto",
            }}
          />
        </IconButton>
        <TextWrapper>
          <Typography>Smart Home</Typography>
        </TextWrapper>
      </StyledToolbar>
    </StyledAppBar>
  );
};

export default TopBar;
