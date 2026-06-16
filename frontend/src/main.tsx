import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./index.css";
import MainPage from "./pages/MainPage.tsx";
import { BrowserRouter, Route, Routes } from "react-router-dom";
import Sensors from "./pages/Sensors.tsx";
import { ROUTES } from "./Routes.ts";
import Layout from "./components/Layout.tsx";
import { Rooms } from "./pages/Rooms.tsx";
import SensorHistory from "./pages/SensorHistory.tsx";
import CssBaseline from "@mui/material/CssBaseline";
import { ThemeProvider } from "@mui/material/styles";
import { setupTheme } from "./theme";

const theme = setupTheme();

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <BrowserRouter>
        <Routes>
          <Route path="/" element={<Layout />}>
            <Route index element={<MainPage />} />
            <Route path={ROUTES.SENSORS} element={<Sensors />} />
            <Route path="/sensors/:id/history" element={<SensorHistory />} />
            <Route path={ROUTES.ROOMS} element={<Rooms />} />
          </Route>
        </Routes>
      </BrowserRouter>
    </ThemeProvider>
  </StrictMode>,
);
