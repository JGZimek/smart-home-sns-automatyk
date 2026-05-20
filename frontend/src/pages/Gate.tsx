import { Button, Typography } from "@mui/material";
import { useEffect, useState } from "react"

interface Gate {
    isOpened: boolean
}

export const Gate = () => {
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
        <>
            {gate ? (
                <>
                    <Typography>Stan bramy: {gate.isOpened ? "otwarta" : "zamknięta"}</Typography>
                    <Button onClick={setGateStatus}>Zmień stan</Button>
                </>)
                : (
                    <Typography>Error fetching gate data</Typography>
                )
            }

        </>
    )
}