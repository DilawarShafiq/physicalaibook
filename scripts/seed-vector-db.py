import sys
import os
import asyncio
import glob
from typing import List

# Add backend to path so we can import app modules
# Assumes script is in /scripts and backend is in /backend
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend'))

from app.services.embedding_service import EmbeddingService
from app.services.qdrant_service import QdrantService
from app.config import settings
from qdrant_client.http import models
import uuid
from llama_index.node_parser import SentenceSplitter
from llama_index.schema import Document

async def main():
    print("Starting Vector DB Seeding...")
    
    embedding_service = EmbeddingService()
    qdrant_service = QdrantService()
    
    # Ensure collection exists
    await qdrant_service.create_collection_if_not_exists()
    
    # Initialize splitter
    splitter = SentenceSplitter(
        chunk_size=settings.CHUNK_SIZE,
        chunk_overlap=settings.chunk_overlap
    )
    
    # Find all markdown files
    docs_dir = os.path.join(os.path.dirname(__file__), '..', 'docs')
    files = glob.glob(f"{docs_dir}/**/*.md", recursive=True)
    
    print(f"Found {len(files)} markdown files.")
    
    total_chunks = 0
    
    for file_path in files:
        print(f"Processing {file_path}...")
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # Create Document
        # Extract relative path for metadata
        rel_path = os.path.relpath(file_path, docs_dir)
        doc = Document(text=content, metadata={"path": rel_path})
        
        # Split into nodes
        nodes = splitter.get_nodes_from_documents([doc])
        
        # Process nodes
        points = []
        for node in nodes:
            # Generate embedding
            vector = await embedding_service.get_embedding(node.text)
            
            # Prepare Qdrant point
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=vector,
                payload={
                    "content": node.text,
                    "metadata": node.metadata,
                    "path": rel_path
                }
            )
            points.append(point)
            
        # Upsert batch
        if points:
            await qdrant_service.client.upsert(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                points=points
            )
            total_chunks += len(points)
            print(f"  Uploaded {len(points)} chunks.")
            
    print(f"Seeding complete. Total chunks: {total_chunks}")

if __name__ == "__main__":
    asyncio.run(main())
