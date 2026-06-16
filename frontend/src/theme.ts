import { createTheme } from "@mui/material";

export function setupTheme() {
  const palette = {
    mode: "dark",
    primary: {
      main: "#90CAF9", // soft blue for dark bg
      light: "#E3F2FD",
      dark: "#42A5F5",
      contrastText: "#0A1929",
    },
    secondary: {
      main: "#FFB74D", // warm accent
      light: "#FFE097",
      dark: "#F57C00",
      contrastText: "#0A1929",
    },
    error: {
      main: "#EF5350",
      light: "#FF867C",
      dark: "#C62828",
      contrastText: "#0A1929",
    },
    warning: {
      main: "#FFB74D",
      light: "#FFE097",
      dark: "#F57C00",
      contrastText: "#0A1929",
    },
    info: {
      main: "#64B5F6",
      light: "#BBDEFB",
      dark: "#1E88E5",
      contrastText: "#0A1929",
    },
    success: {
      main: "#81C784",
      light: "#C8E6C9",
      dark: "#388E3C",
      contrastText: "#0A1929",
    },
    background: {
      default: "#0B1020", // main app background
      paper: "#121826",
    },
    text: {
      primary: "#E3F2FD",
      secondary: "#90A4AE",
      disabled: "#607D8B",
    },
    divider: "#263040",
  } as const;

  return createTheme({
    palette,
  });
}
