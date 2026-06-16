import {
  AppBar,
  Box,
  styled,
  Toolbar,
  Typography,
} from "@mui/material";
import logo from "../assets/logo1.png";

const StyledAppBar = styled(AppBar)(({ theme }) => ({
  backgroundColor: theme.palette.background.paper,
  color: theme.palette.text.primary,
  borderBottom: `1px solid ${theme.palette.divider}`,
  boxShadow: "none",
}));

const StyledToolbar = styled(Toolbar)(({ theme }) => ({
  display: "flex",
  gap: theme.spacing(2),
  minHeight: 72,
  padding: theme.spacing(0, 2),
}));

const TextWrapper = styled(Box)({
  display: "flex",
  alignItems: "center",
  flexGrow: 1,
  gap: 8,
});

export const TopBar = () => {
  return (
    <StyledAppBar position="sticky">
      <StyledToolbar>
        <img
          src={logo}
          alt="Science club logo"
          style={{
            height: "36px",
            width: "auto",
          }}
        />
        <TextWrapper>
          <Typography variant="h3" component="p" fontWeight={700}>
            Smart Home
          </Typography>
        </TextWrapper>
      </StyledToolbar>
    </StyledAppBar>
  );
};

export default TopBar;
