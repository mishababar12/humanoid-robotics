# Auth Service (Better-Auth Integration)

This directory contains the conceptual implementation of an authentication service for the "Physical AI & Humanoid Robotics" textbook. For this project, we are assuming the use of a fictional authentication service called "Better-Auth".

## Better-Auth API Overview

Better-Auth is a fictional, third-party authentication service that provides a simple and secure way to manage users.

### API Endpoints

*   **`POST /signup`**: Creates a new user.
    *   **Request Body**: `{ "email": "user@example.com", "password": "password123" }`
    *   **Response**: `{ "user_id": "12345", "token": "a_jwt_token" }`
*   **`POST /signin`**: Signs in an existing user.
    *   **Request Body**: `{ "email": "user@example.com", "password": "password123" }`
    *   **Response**: `{ "user_id": "12345", "token": "a_jwt_token" }`
*   **`GET /users/{user_id}`**: Retrieves user information.
    *   **Headers**: `{ "Authorization": "Bearer a_jwt_token" }`
    *   **Response**: `{ "user_id": "12345", "email": "user@example.com", "profile": { "background": "software" } }`
*   **`PUT /users/{user_id}`**: Updates user information.
    *   **Headers**: `{ "Authorization": "Bearer a_jwt_token" }`
    *   **Request Body**: `{ "profile": { "background": "hardware" } }`
    *   **Response**: `{ "user_id": "12345", "email": "user@example.com", "profile": { "background": "hardware" } }`

### Integration

The Docusaurus frontend will interact with the Better-Auth API to provide signup and signin functionality. Once a user is signed in, the JWT token will be stored in the browser and used to authenticate subsequent requests.