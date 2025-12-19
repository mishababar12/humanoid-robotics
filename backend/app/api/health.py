from fastapi import APIRouter
from pydantic import BaseModel
from typing import Dict, Any
import cohere
import sys
from pathlib import Path

# Add backend directory to path for imports
backend_dir = Path(__file__).parent.parent.parent
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))

from database.qdrant import qdrant_service
from config import settings
from datetime import datetime


router = APIRouter()


class HealthResponse(BaseModel):
    status: str
    timestamp: str
    services: Dict[str, str]
    details: Dict[str, Any]


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify all services are running
    """
    services_status = {
        "cohere": "disconnected",
        "qdrant": "disconnected",
        "postgres": "disconnected"
    }

    details = {}

    # Check Cohere connection
    try:
        co = cohere.Client(settings.cohere_api_key)
        # Make a simple API call to test connection
        response = co.generate(
            model=settings.cohere_generation_model,
            prompt="test",
            max_tokens=1
        )
        services_status["cohere"] = "connected"
    except Exception as e:
        services_status["cohere"] = f"error: {str(e)}"
        details["cohere_error"] = str(e)

    # Check Qdrant connection
    try:
        collection_info = qdrant_service.get_collection_info()
        if collection_info:
            services_status["qdrant"] = "connected"
            details["qdrant_info"] = collection_info
        else:
            services_status["qdrant"] = "error: unable to get collection info"
    except Exception as e:
        services_status["qdrant"] = f"error: {str(e)}"
        details["qdrant_error"] = str(e)

    # Note: We're not checking Postgres here as it would require importing and initializing the connection
    # In a real implementation, you'd want to check the database connection as well
    services_status["postgres"] = "assumed connected"  # Placeholder

    # Overall status
    overall_status = "healthy" if all("connected" in status for status in services_status.values()) else "degraded"

    return HealthResponse(
        status=overall_status,
        timestamp=datetime.now().isoformat(),
        services=services_status,
        details=details
    )


class ReadyResponse(BaseModel):
    ready: bool
    message: str


@router.get("/ready", response_model=ReadyResponse)
async def readiness_check():
    """
    Readiness check to verify the service is ready to accept traffic
    """
    try:
        # Check if essential services are available
        co = cohere.Client(settings.cohere_api_key)
        # Quick test of Cohere
        co.generate(
            model=settings.cohere_generation_model,
            prompt="ready",
            max_tokens=1
        )

        # Check Qdrant
        qdrant_service.get_collection_info()

        return ReadyResponse(
            ready=True,
            message="Service is ready to accept traffic"
        )
    except Exception as e:
        return ReadyResponse(
            ready=False,
            message=f"Service not ready: {str(e)}"
        )