import fastapi
app = fastapi.FastAPI()


@app.get("/")
async def root():
    return {"message": "Hello World"}

@app.get("/ping")
async def ping():
    return {"ping": "pong"}

