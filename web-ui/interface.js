// Global state
let isRobotEngaged = false;
let currentConfig = {};

// Settings modal functions
function openSettings() {
  const modal = document.getElementById('settingsModal');
  modal.classList.add('show');
  loadConfiguration();
}

function closeSettings() {
  const modal = document.getElementById('settingsModal');
  modal.classList.remove('show');
}

function loadConfiguration() {
  fetch('/api/config')
    .then(response => response.json())
    .then(config => {
      currentConfig = config;
      populateSettingsForm(config);
    })
    .catch(error => {
      console.error('Error loading configuration:', error);
      alert('Error loading configuration');
    });
}

function populateSettingsForm(config) {
  // Robot arms
  document.getElementById('leftArmName').value = config.robot?.left_arm?.name || '';
  document.getElementById('rightArmName').value = config.robot?.right_arm?.name || '';
  
  // Network settings
  document.getElementById('httpsPort').value = config.network?.https_port || '';
  document.getElementById('websocketPort').value = config.network?.websocket_port || '';
  document.getElementById('hostIp').value = config.network?.host_ip || '';
  
  // Control parameters
  document.getElementById('vrScale').value = config.robot?.vr_to_robot_scale || '';
  document.getElementById('sendInterval').value = (config.robot?.send_interval * 1000) || ''; // Convert to ms
}

function restartSystem() {
  if (!confirm('Are you sure you want to restart the system? This will temporarily disconnect all devices.')) {
    return;
  }

  const restartButton = document.getElementById('restartButton');
  restartButton.disabled = true;
  restartButton.textContent = 'Restarting...';

  fetch('/api/restart', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    }
  })
  .then(response => {
    if (response.ok) {
      // Show restart message and close modal
      alert('System is restarting... The page will reload automatically in a few seconds.');
      closeSettings();
      
      // Try to reconnect after a delay
      setTimeout(() => {
        window.location.reload();
      }, 5000);
    } else {
      alert('Failed to restart system. Please restart manually.');
    }
  })
  .catch(error => {
    console.error('Error restarting system:', error);
    alert('Error communicating with server. Please restart manually.');
  })
  .finally(() => {
    restartButton.disabled = false;
    restartButton.textContent = '🔄 Restart System';
  });
}

function saveConfiguration() {
  const form = document.getElementById('settingsForm');
  const formData = new FormData(form);
  
  // Build config object
  const updatedConfig = {
    robot: {
      left_arm: {
        name: formData.get('leftArmName'),
        enabled: true
      },
      right_arm: {
        name: formData.get('rightArmName'),
        enabled: true
      },
      vr_to_robot_scale: parseFloat(formData.get('vrScale')),
      send_interval: parseFloat(formData.get('sendInterval')) / 1000 // Convert from ms
    },
    network: {
      https_port: parseInt(formData.get('httpsPort')),
      websocket_port: parseInt(formData.get('websocketPort')),
      host_ip: formData.get('hostIp')
    }
  };

  const saveButton = document.getElementById('saveButton');
  saveButton.disabled = true;
  saveButton.textContent = 'Saving...';

  fetch('/api/config', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify(updatedConfig)
  })
  .then(response => response.json())
  .then(data => {
    if (data.success) {
      alert('Configuration saved successfully! Use the restart button to apply changes.');
    } else {
      alert('Failed to save configuration: ' + (data.error || 'Unknown error'));
    }
  })
  .catch(error => {
    console.error('Error saving configuration:', error);
    alert('Error saving configuration');
  })
  .finally(() => {
    saveButton.disabled = false;
    saveButton.textContent = '💾 Save Configuration';
  });
}

// Update status indicators
function updateStatus() {
  fetch('/api/status')
    .then(response => response.json())
    .then(data => {
      // Update arm connection indicators (based on device files)
      const leftIndicator = document.getElementById('leftArmStatus');
      const rightIndicator = document.getElementById('rightArmStatus');
      const vrIndicator = document.getElementById('vrStatus');
      
      leftIndicator.className = 'status-indicator' + (data.left_arm_connected ? ' connected' : '');
      rightIndicator.className = 'status-indicator' + (data.right_arm_connected ? ' connected' : '');
      vrIndicator.className = 'status-indicator' + (data.vrConnected ? ' connected' : '');

      // Update robot engagement status
      if (data.robotEngaged !== undefined) {
        isRobotEngaged = data.robotEngaged;
        updateEngagementUI();
      }
    })
    .catch(error => {
      console.error('Error fetching status:', error);
    });
}

function updateEngagementUI() {
  const engageBtn = document.getElementById('robotEngageBtn');
  const engageBtnText = document.getElementById('engageBtnText');
  const engagementStatusText = document.getElementById('engagementStatusText');
  const connectionHint = document.getElementById('connectionHint');
  const connectionWarning = document.getElementById('connectionWarning');

  if (isRobotEngaged) {
    engageBtn.classList.add('disconnect');
    engageBtn.classList.remove('needs-attention');
    engageBtnText.textContent = '🔌 Disconnect Robot';
    engagementStatusText.textContent = 'Motors Engaged';
    engagementStatusText.style.color = '#FFFFFF';
    if (connectionHint) connectionHint.style.display = 'none';
    if (connectionWarning) connectionWarning.classList.remove('show');
  } else {
    engageBtn.classList.remove('disconnect');
    engageBtnText.textContent = '🔌 Connect Robot';
    engagementStatusText.textContent = 'Motors Disengaged';
    engagementStatusText.style.color = '#FFFFFF';
    if (connectionHint) connectionHint.style.display = 'block';
  }
}

function toggleRobotEngagement() {
  const action = isRobotEngaged ? 'disconnect' : 'connect';
  
  fetch('/api/robot', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ action: action })
  })
  .then(response => response.json())
  .then(data => {
    if (data.success) {
      isRobotEngaged = !isRobotEngaged;
      updateEngagementUI();
    } else {
      alert('Failed to ' + action + ' robot: ' + (data.error || 'Unknown error'));
    }
  })
  .catch(error => {
    console.error('Error toggling robot engagement:', error);
    alert('Error communicating with server');
  });
}

// Check if running in VR/AR mode
function isVRMode() {
  return window.navigator.xr && document.fullscreenElement;
}

// Update UI based on device
function updateUIForDevice() {
  const desktopInterface = document.getElementById('desktopInterface');

  if (isVRMode()) {
    desktopInterface.style.display = 'none';
  } else {
    // Always show desktop interface initially - let the start button handle VR entry
    desktopInterface.style.display = 'block';
  }
}

// Initialize
document.addEventListener('DOMContentLoaded', () => {
  updateUIForDevice();
  
  // Start status monitoring
  updateStatus();
  setInterval(updateStatus, 2000); // Update every 2 seconds
  
  // Check A-Frame scene loading
  const scene = document.querySelector('a-scene');
  if (scene) {
    console.log('A-Frame scene element found');
    console.log('Scene hasLoaded:', scene.hasLoaded);
    console.log('Scene is:', scene.is);
    console.log('Scene components:', Object.keys(scene.components || {}));
    
    if (!scene.hasLoaded) {
      console.log('Scene not loaded yet, waiting for loaded event...');
      scene.addEventListener('loaded', () => {
        console.log('✅ A-Frame scene loaded successfully!');
        console.log('Scene renderStarted:', scene.renderStarted);
        console.log('enterVR available:', typeof scene.enterVR);
      });
    } else {
      console.log('✅ A-Frame scene already loaded!');
      console.log('enterVR available:', typeof scene.enterVR);
    }
  } else {
    console.error('❌ A-Frame scene element not found!');
  }
  
  // Handle VR mode changes
  document.addEventListener('fullscreenchange', updateUIForDevice);
  
  // VR session detection
  if (navigator.xr) {
    navigator.xr.addEventListener('sessionstart', () => {
      updateStatus();
      updateUIForDevice();
    });
    
    navigator.xr.addEventListener('sessionend', () => {
      updateStatus();
      updateUIForDevice();
    });
  }

  // Settings form handler
  document.getElementById('settingsForm').addEventListener('submit', (e) => {
    e.preventDefault();
    saveConfiguration();
  });

  // Close modal when clicking outside
  document.getElementById('settingsModal').addEventListener('click', (e) => {
    if (e.target.id === 'settingsModal') {
      closeSettings();
    }
  });
});

// Handle window resize
window.addEventListener('resize', updateUIForDevice);

// Switch between desktop and VR views
function switchToVrView() {
  console.log('Switching to VR view');
  const desktopInterface = document.getElementById('desktopInterface');
  desktopInterface.style.display = 'none';
  
  // Keep a visible background before XR session starts.
  document.body.style.background = 'linear-gradient(135deg, #0f172a 0%, #1f2937 100%)';
  
  // Make sure the A-Frame scene is visible
  const scene = document.querySelector('a-scene');
  if (scene) {
    scene.style.display = 'block';
    scene.style.visibility = 'visible';
  }
  
  // Make sure the start button and instructions are visible
  let startButton = document.getElementById('start-tracking-button');
  let instructionsPanel = document.getElementById('vr-instructions-panel');
  
  if (!startButton) {
    console.log('Start button not found, creating fallback button');
    createFallbackVrButton();
    startButton = document.getElementById('start-tracking-button');
  }
  
  if (startButton) {
    startButton.style.display = 'block';
    console.log('Start button made visible');
  }
  
  if (instructionsPanel) {
    instructionsPanel.style.display = 'block';
  }
  
  showBackToDesktopButton();
}

function switchToDesktopView() {
  console.log('Switching to desktop view');
  const desktopInterface = document.getElementById('desktopInterface');
  desktopInterface.style.display = 'block';
  
  // Restore body background
  document.body.style.background = 'linear-gradient(135deg, #1a1a2e 0%, #16213e 100%)';
  
  // Hide VR elements
  const startButton = document.getElementById('start-tracking-button');
  const instructionsPanel = document.getElementById('vr-instructions-panel');
  
  if (startButton) {
    startButton.style.display = 'none';
  }
  
  if (instructionsPanel) {
    instructionsPanel.style.display = 'none';
  }
  
  hideBackToDesktopButton();
}

function showBackToDesktopButton() {
  let backBtn = document.getElementById('back-to-desktop-button');
  if (!backBtn) {
    backBtn = document.createElement('button');
    backBtn.id = 'back-to-desktop-button';
    backBtn.textContent = '← Back to Desktop View';
    backBtn.style.cssText = `
      position: fixed;
      bottom: 20px;
      left: 50%;
      transform: translateX(-50%);
      padding: 12px 24px;
      font-size: 16px;
      background-color: #666;
      color: white;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      z-index: 9998;
      box-shadow: 0 4px 8px rgba(0,0,0,0.3);
    `;
    backBtn.onclick = switchToDesktopView;
    document.body.appendChild(backBtn);
  }
  backBtn.style.display = 'block';
}

function hideBackToDesktopButton() {
  const backBtn = document.getElementById('back-to-desktop-button');
  if (backBtn) {
    backBtn.style.display = 'none';
  }
}

function createFallbackVrButton() {
  // Also create the instructions panel if the function exists
  if (typeof createVrInstructionsPanel === 'function') {
    createVrInstructionsPanel();
  }

  const startButton = document.createElement('button');
  startButton.id = 'start-tracking-button';
  startButton.textContent = 'Start Controller Tracking';
  startButton.style.cssText = `
    position: fixed;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    padding: 20px 40px;
    font-size: 20px;
    font-weight: bold;
    background-color: #4CAF50;
    color: white;
    border: none;
    border-radius: 8px;
    cursor: pointer;
    z-index: 9999;
    box-shadow: 0 4px 8px rgba(0,0,0,0.3);
  `;
  startButton.onclick = async () => {
    const sceneEl = document.querySelector('a-scene');
    if (sceneEl) {
      // Wait for scene to be fully loaded
      if (!sceneEl.hasLoaded) {
        console.log('Scene not loaded yet, waiting...');
        await new Promise(resolve => {
          sceneEl.addEventListener('loaded', resolve, { once: true });
        });
      }
      
      startButton.textContent = 'Connecting...';
      startButton.disabled = true;
      try {
        // Check if robot is already connected
        const statusResponse = await fetch('/api/status');
        const status = await statusResponse.json();
        
        // Only try to connect robot if not in no-robot mode
        if (status.robotEngaged === false && status.left_arm_connected !== undefined) {
          startButton.textContent = 'Connecting Arms...';
          const connectResponse = await fetch('/api/robot', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ action: 'connect' })
          });
          const connectResult = await connectResponse.json();
          if (!connectResult.success) {
            console.warn('Failed to connect robot arms:', connectResult.error);
            // Don't fail if in no-robot mode
            if (!connectResult.error.includes('interface')) {
              throw new Error(connectResult.error || 'Failed to connect robot arms');
            }
          } else {
            await new Promise(resolve => setTimeout(resolve, 500));
          }
        }
        
        startButton.textContent = 'Starting VR...';
        
        // Check if enterVR method exists
        if (typeof sceneEl.enterVR !== 'function') {
          console.error('enterVR method not found on scene element');
          console.log('Available scene methods:', Object.getOwnPropertyNames(Object.getPrototypeOf(sceneEl)));
          
          // Try alternative: directly request XR session
          if (navigator.xr) {
            console.log('Trying to request XR session directly...');
            try {
              const session = await navigator.xr.requestSession('immersive-vr', {
                optionalFeatures: ['local-floor', 'bounded-floor', 'hand-tracking']
              });
              console.log('XR session created successfully');
              startButton.style.display = 'none';
            } catch (xrErr) {
              throw new Error(`Failed to create XR session: ${xrErr.message}`);
            }
          } else {
            throw new Error('A-Frame VR mode not available. Make sure you are using a WebXR-compatible browser.');
          }
        } else {
          console.log('Calling sceneEl.enterVR()...');
          sceneEl.enterVR();
        }
      } catch (err) {
        alert(`Failed to start: ${err.message}`);
        startButton.textContent = 'Start Controller Tracking';
        startButton.disabled = false;
      }
    } else {
      alert('VR scene not available. Please reload the page.');
    }
  };
  document.body.appendChild(startButton);
} 
