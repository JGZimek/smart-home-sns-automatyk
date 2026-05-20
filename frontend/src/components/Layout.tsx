import { Outlet } from "react-router-dom";
import TopBar from "./TopBar"

export const Layout = () => {
    return (
        <>
            <TopBar />
            <Outlet />
        </>
    )
}

export default Layout;