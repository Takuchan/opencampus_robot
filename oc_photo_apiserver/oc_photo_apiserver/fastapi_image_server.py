from fastapi import FastAPI
from fastapi.responses import FileResponse
import os

app = FastAPI()
image_dir = 'oc_saved_images'

@app.get("/images")
def list_images():
    """ 保存されている画像の一覧を取得 """
    if not os.path.exists(image_dir):
        return {"error": "No images found"}
    files = os.listdir(image_dir)
    return {"images": files}

@app.get("/images/{filename}")
def get_image(filename: str):
    """ 指定された画像を返す """
    file_path = os.path.join(image_dir, filename)
    if not os.path.exists(file_path):
        return {"error": "File not found"}
    return FileResponse(file_path)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
