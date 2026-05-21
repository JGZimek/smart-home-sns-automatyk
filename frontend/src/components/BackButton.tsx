import { Button } from "@mui/material"
import backIcon from "../assets/back.png"
import { useNavigate } from "react-router-dom"

type BackButtonProps = {
    destination: string
}

export const BackButton = ({ destination }: BackButtonProps) => {
    const navigate = useNavigate()
    return (
        <Button startIcon={<img src={backIcon} />} onClick={() => navigate(destination)} >
            Back
        </Button>
    )
}