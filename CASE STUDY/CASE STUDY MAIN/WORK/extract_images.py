import zipfile
import os
from pathlib import Path

def extract_pptx_media(pptx_path, output_dir):
    print(f"Extracting media from {pptx_path}...")
    if not os.path.exists(pptx_path):
        print(f"File not found: {pptx_path}")
        return
    
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    with zipfile.ZipFile(pptx_path, 'r') as archive:
        for file_info in archive.infolist():
            if file_info.filename.startswith('ppt/media/'):
                filename = os.path.basename(file_info.filename)
                if filename:
                    output_path = os.path.join(output_dir, filename)
                    with open(output_path, 'wb') as f_out:
                        f_out.write(archive.read(file_info.filename))
                    print(f"Extracted: {filename}")

def main():
    root = "/home/ganeshna/person_follower_robot_project/CASE STUDY/CASE STUDY MAIN"
    
    # Extract presentation 1
    pptx1 = os.path.join(root, "PRESENTATION", "FINAL PRESENTATION CASE STUDY OF ROS .pptx")
    out1 = os.path.join(root, "IMAGES", "presentation1_media")
    extract_pptx_media(pptx1, out1)
    
    # Extract presentation 2
    pptx2 = os.path.join(root, "PRESENTATION", "AI-Based Object Following Robot using TurtleBot3.pptx")
    out2 = os.path.join(root, "IMAGES", "presentation2_media")
    extract_pptx_media(pptx2, out2)

if __name__ == '__main__':
    main()
