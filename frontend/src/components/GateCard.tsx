import { Button, Card, CardContent, CardMedia, styled, Typography } from "@mui/material"
import gateIcon from "../assets/gate.png"
import { useEffect, useState } from "react"

interface Gate {
    isOpened: boolean
}

const CardWrapper = styled(Card)(() => ({
    display: "flex",
    alignItems: "center",
    justifyContent: "center"
}))

export const GateCard = () => {

    const [gate, setGate] = useState<Gate>()

    useEffect(() => {
        fetch("http://localhost:3000/gate")
            .then((res) => res.json())
            .then((json) => {
                setGate(json);
            });
    }, []);


    function setGateStatus() {
        if (!gate) return;

        fetch("http://localhost:3000/gate", {
            method: "PATCH",
            headers: {
                "Content-Type": "application/json",
            },
            body: JSON.stringify({
                ...gate,
                isOpened: !gate.isOpened
            })
        })
            .then((res) => res.json())
            .then((json) => {
                setGate(json);
            });
    }

    return (
        <CardWrapper>
            <CardMedia
                sx={{
                    height: 30,
                    width: 30
                }}
                image={gateIcon}
                title={"Gate image"}
            />
            {gate ? (
                <>
                    <CardContent>
                        <Typography>Gate status: {gate.isOpened ? "opened" : "closed"} </Typography>
                    </CardContent>
                    <Button onClick={setGateStatus}> {gate.isOpened ? "Close" : "Open"} gate </Button>
                </>
            ) : (
                <Typography>Error fetching gate data</Typography>
            )
            }
        </CardWrapper>
    )
}

export default GateCard