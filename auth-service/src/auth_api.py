from fastapi import FastAPI, HTTPException

# This is a conceptual example of an authentication API that mocks the Better-Auth service.

app = FastAPI(
    title="Auth Service",
    description="A mock authentication service for the Physical AI & Humanoid Robotics textbook.",
    version="0.1.0",
)

# In-memory database for storing users
users = {}

@app.post("/signup")
def signup(email: str, password: str):
    if email in users:
        raise HTTPException(status_code=400, detail="Email already registered")
    user_id = str(len(users) + 1)
    users[email] = {"user_id": user_id, "password": password, "profile": {}}
    return {"user_id": user_id, "token": "a_jwt_token"}

@app.post("/signin")
def signin(email: str, password: str):
    if email not in users or users[email]["password"] != password:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    return {"user_id": users[email]["user_id"], "token": "a_jwt_token"}

@app.get("/users/{user_id}")
def get_user(user_id: str):
    for email, user in users.items():
        if user["user_id"] == user_id:
            return {"user_id": user_id, "email": email, "profile": user["profile"]}
    raise HTTPException(status_code=404, detail="User not found")

@app.put("/users/{user_id}")
def update_user(user_id: str, profile: dict):
    for email, user in users.items():
        if user["user_id"] == user_id:
            user["profile"] = profile
            return {"user_id": user_id, "email": email, "profile": user["profile"]}
    raise HTTPException(status_code=404, detail="User not found")