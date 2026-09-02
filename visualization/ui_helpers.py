"""Small UI helper utilities for formatting and layout."""
from typing import Dict, Any

def telemetry_text_block(formatted: Dict[str, str]) -> str:
    lines = [f"{k}: {v}" for k, v in formatted.items()]
    return "\n".join(lines)
