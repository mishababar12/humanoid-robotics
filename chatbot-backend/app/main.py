from fastapi import FastAPI
from .routers import chatbot

app = FastAPI(
    title="Physical AI & Humanoid Robotics Chatbot",
    description="A RAG chatbot for the Physical AI & Humanoid Robotics textbook.",
    version="0.1.0",
)

app.include_router(chatbot.router)

@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.get("/health")
def health_check():
    return {"status": "ok"}
