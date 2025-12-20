@echo off
REM Fix: Push ONLY backend folder to HF Spaces
REM This ensures Dockerfile is at root level of HF Space

echo.
echo ========================================
echo   Pushing Backend to Hugging Face
echo ========================================
echo.

cd /d "%~dp0"

echo Current directory: %CD%
echo.

REM Check if Dockerfile exists
if not exist "Dockerfile" (
    echo [ERROR] Dockerfile not found!
    echo Make sure you're in the backend folder
    pause
    exit /b 1
)

echo [OK] Dockerfile found
echo.

REM Check if HF remote exists
git remote get-url hf >nul 2>&1
if errorlevel 1 (
    echo [ERROR] HF remote not found!
    echo.
    echo Add it first with:
    echo git remote add hf https://huggingface.co/spaces/Mishababar/rag-chatbot-backend
    echo.
    pause
    exit /b 1
)

echo [OK] HF remote configured:
git remote get-url hf
echo.

echo [STEP 1] Adding all files...
git add .
echo.

echo [STEP 2] Creating commit...
git commit -m "Deploy backend to HF Spaces"
echo.

echo [STEP 3] Pushing to HF Spaces...
echo.
echo You'll need to enter:
echo   Username: Mishababar
echo   Password: [Your HF access token from https://huggingface.co/settings/tokens]
echo.
pause

git push hf HEAD:main -f

echo.
echo ========================================
echo   Push Complete!
echo ========================================
echo.
echo Now check your HF Space:
echo https://huggingface.co/spaces/Mishababar/rag-chatbot-backend
echo.
echo Go to "Logs" tab to see build progress
echo.
pause
