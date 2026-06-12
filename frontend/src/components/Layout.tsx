import { Outlet } from "react-router-dom";
import TopBar from "./TopBar";
import { Box, styled } from "@mui/material";

const ContentWrapper = styled(Box)(({ theme }) => ({
  margin: theme.spacing(2, 1),
}));

export const Layout = () => {
  return (
    <>
      <TopBar />
      <ContentWrapper>
        <Outlet />
      </ContentWrapper>
    </>
  );
};

export default Layout;
