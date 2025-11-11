# 🚀 Hugging Face Spaces Deployment Guide

This guide will help you deploy your 8-Puzzle AI Solver to Hugging Face Spaces.

## Prerequisites

1. A Hugging Face account (sign up at https://huggingface.co/join)
2. Your project files ready (already done! ✅)

## Deployment Steps

### Option 1: Deploy via Web Interface (Easiest)

1. **Go to Hugging Face Spaces**
   - Visit https://huggingface.co/spaces
   - Click "Create new Space"

2. **Configure Your Space**
   - **Owner**: Select `Ab-Romia`
   - **Space name**: `8-Puzzle-AI` (or any name you prefer)
   - **License**: MIT
   - **SDK**: Select "Gradio"
   - **Visibility**: Public (to showcase your work!)

3. **Upload Your Files**
   Click "Files" and upload these files:
   - `app.py` (main application)
   - `requirements.txt` (dependencies)
   - `README.md` (project documentation)
   - `DONE/` folder (all Python files inside)
     - `__init__.py`
     - `state.py`
     - `search_algorithms.py`
     - `heuristics.py`
     - `data_structures.py`
     - `visualize.py`

4. **Wait for Build**
   - Hugging Face will automatically detect the Gradio app
   - It will install dependencies from `requirements.txt`
   - The app will be live in 2-3 minutes!

5. **Your Space URL**
   - `https://huggingface.co/spaces/Ab-Romia/8-Puzzle-AI`

### Option 2: Deploy via Git (Advanced)

1. **Clone your Space repository**
   ```bash
   git clone https://huggingface.co/spaces/Ab-Romia/8-Puzzle-AI
   cd 8-Puzzle-AI
   ```

2. **Copy your project files**
   ```bash
   cp -r /home/user/8_Puzzle-AI/* .
   ```

3. **Commit and push**
   ```bash
   git add .
   git commit -m "Initial deployment of 8-Puzzle AI Solver"
   git push
   ```

## Post-Deployment

### 1. Test Your App
Visit your Space URL and test:
- Random puzzle generation
- All 5 algorithms
- Custom puzzle input
- Solution visualization

### 2. Share Your Work
Add badges to your GitHub README:
```markdown
[![Hugging Face Spaces](https://img.shields.io/badge/%F0%9F%A4%97%20Hugging%20Face-Spaces-blue)](https://huggingface.co/spaces/Ab-Romia/8-Puzzle-AI)
```

### 3. Customize (Optional)
You can add to your Space:
- A custom thumbnail (screenshot of your app)
- Tags for discoverability
- A more detailed Space description

## Troubleshooting

### Build Failed?
- Check that all files are uploaded
- Verify `requirements.txt` has correct package names
- Check the build logs in the Space settings

### App Not Loading?
- Ensure `app.py` ends with `demo.launch()`
- Check for import errors in the logs
- Verify Python version compatibility

### Slow Performance?
- Hugging Face free tier has limited resources
- Consider adding timeout limits for DFS
- A* algorithms are fastest and recommended

## File Structure on Hugging Face

```
8-Puzzle-AI/                 (Your Space root)
├── app.py                   # Main Gradio app
├── requirements.txt         # Python dependencies
├── README.md               # Shown on Space page
└── DONE/                   # Algorithm package
    ├── __init__.py
    ├── state.py
    ├── search_algorithms.py
    ├── heuristics.py
    ├── data_structures.py
    └── visualize.py
```

## Important Notes

1. **No Need for Graphviz**: The app is designed to work without graphviz on Hugging Face
2. **Startup Time**: First load may take 10-20 seconds as dependencies install
3. **Free Tier**: Your Space will sleep after inactivity but wakes up automatically
4. **Updates**: Push to the Space repo to update your app

## Next Steps

1. Deploy to Hugging Face using Option 1 or 2
2. Test thoroughly
3. Share on social media (LinkedIn, Twitter, etc.)
4. Add to your portfolio
5. Consider upgrading to paid tier for better performance

## Resources

- [Hugging Face Spaces Documentation](https://huggingface.co/docs/hub/spaces)
- [Gradio Documentation](https://gradio.app/docs/)
- [Your GitHub Repository](https://github.com/Ab-Romia/8_Puzzle-AI)

---

Good luck with your deployment! 🚀
