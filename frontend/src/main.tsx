import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./index.css";
import MainPage from "./pages/MainPage.tsx";
import { BrowserRouter, Route, Routes } from "react-router-dom";
import Sensors from "./pages/Sensors.tsx";
import { ROUTES } from "./Routes.ts";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<MainPage />} />
        <Route path={ROUTES.SENSORS} element={<Sensors />} />
      </Routes>
    </BrowserRouter>
  </StrictMode>,
);
