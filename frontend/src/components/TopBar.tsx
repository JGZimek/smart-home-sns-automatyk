import { Box, AppBar, IconButton, Typography, Toolbar } from "@mui/material"
import logo from '../assets/logo1.png'
import { useNavigate } from "react-router-dom";

export const TopBar = () => {
    const navigate = useNavigate();
    return (
        <AppBar
            position="sticky"
            sx={{
                backgroundColor: "lightgray",
                color: "black"
            }}>
            <Toolbar sx={{ display: "flex", gap: "15px", padding: 0 }}>
                <IconButton
                    onClick={() => navigate("/")}
                >
                    <img
                        src={logo}
                        alt="Science club logo"
                        style={{
                            height: '30px',
                            width: 'auto',
                        }}
                    />
                </IconButton>
                <Box sx={{
                    display: "flex",
                    justifyContent: "center",
                    width: "100%",
                }}>
                    <Typography>Smart Home</Typography>
                </Box>
            </Toolbar>

        </AppBar>
    )
}

export default TopBar;