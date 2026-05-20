import { Box, Container, IconButton, Typography } from "@mui/material"
import logo from '../assets/logo1.png'
import { useNavigate } from "react-router-dom";

export const TopBar = () => {
    const navigate = useNavigate();
    return (
        <Container sx={{
            display: "flex",
            alignItems: "center",
            gap: "15px",
            padding: 0,
            backgroundColor: "lightgray"

        }}>
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
        </Container>
    )
}

export default TopBar;