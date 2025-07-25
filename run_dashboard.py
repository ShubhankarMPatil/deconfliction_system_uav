#!/usr/bin/env python3
"""
Enhanced UAV Deconfliction Dashboard
Run this script to start the dashboard server
"""

import sys
import os
import webbrowser
from threading import Timer

# Add the project root to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def open_browser():
    """Open the dashboard in default browser after a short delay"""
    webbrowser.open('http://localhost:8050')

def main():
    try:
        print("🛰️ Starting Enhanced UAV Deconfliction Dashboard...")
        print("📊 Loading mission data and initializing components...")
        
        # Import and run the dashboard
        from dashboard.enhanced_dashboard import app
        
        print("✅ Dashboard initialized successfully!")
        print("🌐 Starting web server...")
        print("📱 Dashboard will be available at: http://localhost:8050")
        print("🚀 Opening browser automatically...")
        
        # Open browser after 2 seconds
        Timer(2.0, open_browser).start()
        
        # Start the dashboard
        app.run(debug=False, host='127.0.0.1', port=8050)
        
    except KeyboardInterrupt:
        print("\n🛑 Dashboard stopped by user")
    except Exception as e:
        print(f"❌ Error starting dashboard: {e}")
        print("📝 Please check that all dependencies are installed:")
        print("   - dash")
        print("   - dash-bootstrap-components") 
        print("   - plotly")
        print("   - numpy")
        print("   - scipy")

if __name__ == "__main__":
    main()
