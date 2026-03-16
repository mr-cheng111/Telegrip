// Wait for A-Frame scene to load

// Create debug overlay for VR headset
function createDebugOverlay() {
    const debugDiv = document.createElement('div');
    debugDiv.id = 'vr-debug-overlay';
    debugDiv.style.cssText = `
        position: fixed;
        top: 10px;
        left: 10px;
        background: rgba(0, 0, 0, 0.8);
        color: #0f0;
        font-family: monospace;
        font-size: 12px;
        padding: 10px;
        z-index: 99999;
        max-width: 400px;
        max-height: 300px;
        overflow-y: auto;
        border: 2px solid #0f0;
        border-radius: 5px;
    `;
    document.body.appendChild(debugDiv);
    return debugDiv;
}

let debugOverlay = null;
function debugLog(message) {
    console.log(message);
    if (!debugOverlay) {
        debugOverlay = createDebugOverlay();
    }
    const time = new Date().toLocaleTimeString();
    debugOverlay.innerHTML += `[${time}] ${message}<br>`;
    debugOverlay.scrollTop = debugOverlay.scrollHeight;
}

AFRAME.registerComponent('controller-updater', {
  init: function () {
    debugLog("✅ Controller updater component initialized.");
    console.log("Controller updater component initialized.");
    // Controllers are enabled

    this.leftHand = document.querySelector('#leftHand');
    this.rightHand = document.querySelector('#rightHand');
    this.leftHandInfoText = document.querySelector('#leftHandInfo');
    this.rightHandInfoText = document.querySelector('#rightHandInfo');

    // --- WebSocket Setup ---
    this.websocket = null;
    this.leftGripDown = false;
    this.rightGripDown = false;
    this.leftTriggerDown = false;
    this.rightTriggerDown = false;
    this.leftXDown = false;
    this.rightADown = false;

    // --- Status reporting ---
    this.lastStatusUpdate = 0;
    this.statusUpdateInterval = 5000; // 5 seconds

    // --- Relative rotation tracking ---
    this.leftGripInitialRotation = null;
    this.rightGripInitialRotation = null;
    this.leftRelativeRotation = { x: 0, y: 0, z: 0 };
    this.rightRelativeRotation = { x: 0, y: 0, z: 0 };

    // --- Quaternion-based Z-axis rotation tracking ---
    this.leftGripInitialQuaternion = null;
    this.rightGripInitialQuaternion = null;
    this.leftZAxisRotation = 0;
    this.rightZAxisRotation = 0;

    // --- Get hostname dynamically ---
    const serverHostname = window.location.hostname;
    const websocketPort = 8442; // Make sure this matches controller_server.py
    const websocketUrl = `wss://${serverHostname}:${websocketPort}`;
    debugLog(`🔌 Connecting to: ${websocketUrl}`);
    console.log(`Attempting WebSocket connection to: ${websocketUrl}`);
    // !!! IMPORTANT: Replace 'YOUR_LAPTOP_IP' with the actual IP address of your laptop !!!
    // const websocketUrl = 'ws://YOUR_LAPTOP_IP:8442';
    try {
      this.websocket = new WebSocket(websocketUrl);
      this.websocket.onopen = (event) => {
        debugLog(`✅ WebSocket connected!`);
        console.log(`WebSocket connected to ${websocketUrl}`);
        this.reportVRStatus(true);
      };
      this.websocket.onerror = (event) => {
        // More detailed error logging
        debugLog(`❌ WebSocket Error: ${event.type}`);
        console.error(`WebSocket Error: Event type: ${event.type}`, event);
        this.reportVRStatus(false);
      };
      this.websocket.onclose = (event) => {
        debugLog(`❌ WebSocket closed: ${event.code}`);
        console.log(`WebSocket disconnected from ${websocketUrl}. Clean close: ${event.wasClean}, Code: ${event.code}, Reason: '${event.reason}'`);
        // Attempt to log specific error if available (might be limited by browser security)
        if (!event.wasClean) {
          console.error('WebSocket closed unexpectedly.');
        }
        this.websocket = null; // Clear the reference
        this.reportVRStatus(false);
      };
      this.websocket.onmessage = (event) => {
        console.log(`WebSocket message received: ${event.data}`); // Log any messages from server
      };
    } catch (error) {
        debugLog(`❌ WebSocket error: ${error.message}`);
        console.error(`Failed to create WebSocket connection to ${websocketUrl}:`, error);
        this.reportVRStatus(false);
    }
    // --- End WebSocket Setup ---

    // --- VR Status Reporting Function ---
    this.reportVRStatus = (connected) => {
      // Update global status if available (for desktop interface)
      if (typeof updateStatus === 'function') {
        updateStatus({ vrConnected: connected });
      }
      
      // Also try to notify parent window if in iframe
      try {
        if (window.parent && window.parent !== window) {
          window.parent.postMessage({
            type: 'vr_status',
            connected: connected
          }, '*');
        }
      } catch (e) {
        // Ignore cross-origin errors
      }
    };

    if (!this.leftHand || !this.rightHand || !this.leftHandInfoText || !this.rightHandInfoText) {
      console.error("Controller or text entities not found!");
      // Check which specific elements are missing
      if (!this.leftHand) console.error("Left hand entity not found");
      if (!this.rightHand) console.error("Right hand entity not found");
      if (!this.leftHandInfoText) console.error("Left hand info text not found");
      if (!this.rightHandInfoText) console.error("Right hand info text not found");
      return;
    }

    // Keep controller entities visible for custom axis/text rendering.
    this.leftHand.setAttribute('visible', true);
    this.rightHand.setAttribute('visible', true);

    // Apply initial rotation to combined text elements
    const textRotation = '-90 0 0'; // Rotate -90 degrees around X-axis
    if (this.leftHandInfoText) this.leftHandInfoText.setAttribute('rotation', textRotation);
    if (this.rightHandInfoText) this.rightHandInfoText.setAttribute('rotation', textRotation);

    // --- Create axis indicators ---
    this.createAxisIndicators();

    // --- Helper function to send grip release message ---
    this.sendGripRelease = (hand) => {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        const releaseMessage = {
          hand: hand,
          gripReleased: true
        };
        this.websocket.send(JSON.stringify(releaseMessage));
        console.log(`Sent grip release for ${hand} hand`);
      }
    };

    // --- Helper function to send trigger release message ---
    this.sendTriggerRelease = (hand) => {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        const releaseMessage = {
          hand: hand,
          triggerReleased: true
        };
        this.websocket.send(JSON.stringify(releaseMessage));
        console.log(`Sent trigger release for ${hand} hand`);
      }
    };

    // --- Helper function to calculate relative rotation ---
    this.calculateRelativeRotation = (currentRotation, initialRotation) => {
      return {
        x: currentRotation.x - initialRotation.x,
        y: currentRotation.y - initialRotation.y,
        z: currentRotation.z - initialRotation.z
      };
    };

    // --- Helper function to calculate Z-axis rotation from quaternions ---
    this.calculateZAxisRotation = (currentQuaternion, initialQuaternion) => {
      // Calculate relative quaternion (from initial to current)
      const relativeQuat = new THREE.Quaternion();
      relativeQuat.multiplyQuaternions(currentQuaternion, initialQuaternion.clone().invert());
      
      // Get the controller's current forward direction (local Z-axis in world space)
      const forwardDirection = new THREE.Vector3(0, 0, 1);
      forwardDirection.applyQuaternion(currentQuaternion);
      
      // Convert relative quaternion to axis-angle representation
      const angle = 2 * Math.acos(Math.abs(relativeQuat.w));
      
      // Handle case where there's no rotation (avoid division by zero)
      if (angle < 0.0001) {
        return 0;
      }
      
      // Get the rotation axis
      const sinHalfAngle = Math.sqrt(1 - relativeQuat.w * relativeQuat.w);
      const rotationAxis = new THREE.Vector3(
        relativeQuat.x / sinHalfAngle,
        relativeQuat.y / sinHalfAngle,
        relativeQuat.z / sinHalfAngle
      );
      
      // Project the rotation axis onto the forward direction to get the component
      // of rotation around the forward axis
      const projectedComponent = rotationAxis.dot(forwardDirection);
      
      // The rotation around the forward axis is the angle times the projection
      const forwardRotation = angle * projectedComponent;
      
      // Convert to degrees and handle the sign properly
      let degrees = THREE.MathUtils.radToDeg(forwardRotation);
      
      // Normalize to -180 to +180 range to avoid sudden jumps
      while (degrees > 180) degrees -= 360;
      while (degrees < -180) degrees += 360;
      
      return degrees;
    };

    // --- Modify Event Listeners ---
    this.leftHand.addEventListener('triggerdown', (evt) => {
        console.log('Left Trigger Pressed');
        this.leftTriggerDown = true;
    });
    this.leftHand.addEventListener('triggerup', (evt) => {
        console.log('Left Trigger Released');
        this.leftTriggerDown = false;
        this.sendTriggerRelease('left'); // Send trigger release message
    });
    this.leftHand.addEventListener('gripdown', (evt) => {
        console.log('Left Grip Pressed');
        this.leftGripDown = true; // Set grip state
        
        // Store initial rotation for relative tracking
        if (this.leftHand.object3D.visible) {
          const leftRotEuler = this.leftHand.object3D.rotation;
          this.leftGripInitialRotation = {
            x: THREE.MathUtils.radToDeg(leftRotEuler.x),
            y: THREE.MathUtils.radToDeg(leftRotEuler.y),
            z: THREE.MathUtils.radToDeg(leftRotEuler.z)
          };
          
          // Store initial quaternion for Z-axis rotation tracking
          this.leftGripInitialQuaternion = this.leftHand.object3D.quaternion.clone();
          
          console.log('Left grip initial rotation:', this.leftGripInitialRotation);
          console.log('Left grip initial quaternion:', this.leftGripInitialQuaternion);
        }
    });
    this.leftHand.addEventListener('gripup', (evt) => { // Add gripup listener
        console.log('Left Grip Released');
        this.leftGripDown = false; // Reset grip state
        this.leftGripInitialRotation = null; // Reset initial rotation
        this.leftGripInitialQuaternion = null; // Reset initial quaternion
        this.leftRelativeRotation = { x: 0, y: 0, z: 0 }; // Reset relative rotation
        this.leftZAxisRotation = 0; // Reset Z-axis rotation
        this.sendGripRelease('left'); // Send grip release message
    });
    this.leftHand.addEventListener('xbuttondown', (evt) => {
        this.leftXDown = true;
    });
    this.leftHand.addEventListener('xbuttonup', (evt) => {
        this.leftXDown = false;
    });
    this.leftHand.addEventListener('xbuttonchanged', (evt) => {
        const state = evt && evt.detail && evt.detail.state ? evt.detail.state : null;
        if (state) {
          this.leftXDown = !!(state.pressed || state.value > 0.5);
        }
    });

    this.rightHand.addEventListener('triggerdown', (evt) => {
        console.log('Right Trigger Pressed');
        this.rightTriggerDown = true;
    });
    this.rightHand.addEventListener('triggerup', (evt) => {
        console.log('Right Trigger Released');
        this.rightTriggerDown = false;
        this.sendTriggerRelease('right'); // Send trigger release message
    });
    this.rightHand.addEventListener('gripdown', (evt) => {
        console.log('Right Grip Pressed');
        this.rightGripDown = true; // Set grip state
        
        // Store initial rotation for relative tracking
        if (this.rightHand.object3D.visible) {
          const rightRotEuler = this.rightHand.object3D.rotation;
          this.rightGripInitialRotation = {
            x: THREE.MathUtils.radToDeg(rightRotEuler.x),
            y: THREE.MathUtils.radToDeg(rightRotEuler.y),
            z: THREE.MathUtils.radToDeg(rightRotEuler.z)
          };
          
          // Store initial quaternion for Z-axis rotation tracking
          this.rightGripInitialQuaternion = this.rightHand.object3D.quaternion.clone();
          
          console.log('Right grip initial rotation:', this.rightGripInitialRotation);
          console.log('Right grip initial quaternion:', this.rightGripInitialQuaternion);
        }
    });
    this.rightHand.addEventListener('gripup', (evt) => { // Add gripup listener
        console.log('Right Grip Released');
        this.rightGripDown = false; // Reset grip state
        this.rightGripInitialRotation = null; // Reset initial rotation
        this.rightGripInitialQuaternion = null; // Reset initial quaternion
        this.rightRelativeRotation = { x: 0, y: 0, z: 0 }; // Reset relative rotation
        this.rightZAxisRotation = 0; // Reset Z-axis rotation
        this.sendGripRelease('right'); // Send grip release message
    });
    this.rightHand.addEventListener('abuttondown', (evt) => {
        this.rightADown = true;
    });
    this.rightHand.addEventListener('abuttonup', (evt) => {
        this.rightADown = false;
    });
    this.rightHand.addEventListener('abuttonchanged', (evt) => {
        const state = evt && evt.detail && evt.detail.state ? evt.detail.state : null;
        if (state) {
          this.rightADown = !!(state.pressed || state.value > 0.5);
        }
    });
    // --- End Modify Event Listeners ---

  },

  createAxisIndicators: function() {
    // Create XYZ axis indicators for both controllers
    
    // Left Controller Axes
    // X-axis (Red)
    const leftXAxis = document.createElement('a-cylinder');
    leftXAxis.setAttribute('id', 'leftXAxis');
    leftXAxis.setAttribute('height', '0.08');
    leftXAxis.setAttribute('radius', '0.003');
    leftXAxis.setAttribute('color', '#ff0000'); // Red for X
    leftXAxis.setAttribute('position', '0.04 0 0');
    leftXAxis.setAttribute('rotation', '0 0 90'); // Rotate to point along X-axis
    this.leftHand.appendChild(leftXAxis);

    const leftXTip = document.createElement('a-cone');
    leftXTip.setAttribute('height', '0.015');
    leftXTip.setAttribute('radius-bottom', '0.008');
    leftXTip.setAttribute('radius-top', '0');
    leftXTip.setAttribute('color', '#ff0000');
    leftXTip.setAttribute('position', '0.055 0 0');
    leftXTip.setAttribute('rotation', '0 0 90');
    this.leftHand.appendChild(leftXTip);

    // Y-axis (Green) - Up
    const leftYAxis = document.createElement('a-cylinder');
    leftYAxis.setAttribute('id', 'leftYAxis');
    leftYAxis.setAttribute('height', '0.08');
    leftYAxis.setAttribute('radius', '0.003');
    leftYAxis.setAttribute('color', '#00ff00'); // Green for Y
    leftYAxis.setAttribute('position', '0 0.04 0');
    leftYAxis.setAttribute('rotation', '0 0 0'); // Default up orientation
    this.leftHand.appendChild(leftYAxis);

    const leftYTip = document.createElement('a-cone');
    leftYTip.setAttribute('height', '0.015');
    leftYTip.setAttribute('radius-bottom', '0.008');
    leftYTip.setAttribute('radius-top', '0');
    leftYTip.setAttribute('color', '#00ff00');
    leftYTip.setAttribute('position', '0 0.055 0');
    this.leftHand.appendChild(leftYTip);

    // Z-axis (Blue) - Forward
    const leftZAxis = document.createElement('a-cylinder');
    leftZAxis.setAttribute('id', 'leftZAxis');
    leftZAxis.setAttribute('height', '0.08');
    leftZAxis.setAttribute('radius', '0.003');
    leftZAxis.setAttribute('color', '#0000ff'); // Blue for Z
    leftZAxis.setAttribute('position', '0 0 0.04');
    leftZAxis.setAttribute('rotation', '90 0 0'); // Rotate to point along Z-axis
    this.leftHand.appendChild(leftZAxis);

    const leftZTip = document.createElement('a-cone');
    leftZTip.setAttribute('height', '0.015');
    leftZTip.setAttribute('radius-bottom', '0.008');
    leftZTip.setAttribute('radius-top', '0');
    leftZTip.setAttribute('color', '#0000ff');
    leftZTip.setAttribute('position', '0 0 0.055');
    leftZTip.setAttribute('rotation', '90 0 0');
    this.leftHand.appendChild(leftZTip);

    // Right Controller Axes
    // X-axis (Red)
    const rightXAxis = document.createElement('a-cylinder');
    rightXAxis.setAttribute('id', 'rightXAxis');
    rightXAxis.setAttribute('height', '0.08');
    rightXAxis.setAttribute('radius', '0.003');
    rightXAxis.setAttribute('color', '#ff0000'); // Red for X
    rightXAxis.setAttribute('position', '0.04 0 0');
    rightXAxis.setAttribute('rotation', '0 0 90'); // Rotate to point along X-axis
    this.rightHand.appendChild(rightXAxis);

    const rightXTip = document.createElement('a-cone');
    rightXTip.setAttribute('height', '0.015');
    rightXTip.setAttribute('radius-bottom', '0.008');
    rightXTip.setAttribute('radius-top', '0');
    rightXTip.setAttribute('color', '#ff0000');
    rightXTip.setAttribute('position', '0.055 0 0');
    rightXTip.setAttribute('rotation', '0 0 90');
    this.rightHand.appendChild(rightXTip);

    // Y-axis (Green) - Up
    const rightYAxis = document.createElement('a-cylinder');
    rightYAxis.setAttribute('id', 'rightYAxis');
    rightYAxis.setAttribute('height', '0.08');
    rightYAxis.setAttribute('radius', '0.003');
    rightYAxis.setAttribute('color', '#00ff00'); // Green for Y
    rightYAxis.setAttribute('position', '0 0.04 0');
    rightYAxis.setAttribute('rotation', '0 0 0'); // Default up orientation
    this.rightHand.appendChild(rightYAxis);

    const rightYTip = document.createElement('a-cone');
    rightYTip.setAttribute('height', '0.015');
    rightYTip.setAttribute('radius-bottom', '0.008');
    rightYTip.setAttribute('radius-top', '0');
    rightYTip.setAttribute('color', '#00ff00');
    rightYTip.setAttribute('position', '0 0.055 0');
    this.rightHand.appendChild(rightYTip);

    // Z-axis (Blue) - Forward
    const rightZAxis = document.createElement('a-cylinder');
    rightZAxis.setAttribute('id', 'rightZAxis');
    rightZAxis.setAttribute('height', '0.08');
    rightZAxis.setAttribute('radius', '0.003');
    rightZAxis.setAttribute('color', '#0000ff'); // Blue for Z
    rightZAxis.setAttribute('position', '0 0 0.04');
    rightZAxis.setAttribute('rotation', '90 0 0'); // Rotate to point along Z-axis
    this.rightHand.appendChild(rightZAxis);

    const rightZTip = document.createElement('a-cone');
    rightZTip.setAttribute('height', '0.015');
    rightZTip.setAttribute('radius-bottom', '0.008');
    rightZTip.setAttribute('radius-top', '0');
    rightZTip.setAttribute('color', '#0000ff');
    rightZTip.setAttribute('position', '0 0 0.055');
    rightZTip.setAttribute('rotation', '90 0 0');
    this.rightHand.appendChild(rightZTip);

    console.log('XYZ axis indicators created for both controllers (RGB for XYZ)');
  },

  _readGamepadButtonPressed: function(handEntity, preferredIndex) {
    try {
      const tracked = handEntity && handEntity.components ? handEntity.components['tracked-controls'] : null;
      const controller = tracked && tracked.controller ? tracked.controller : null;
      const gamepad = controller && controller.gamepad ? controller.gamepad : null;
      const buttons = gamepad && gamepad.buttons ? gamepad.buttons : null;
      if (!buttons || buttons.length === 0) return null;
      if (preferredIndex >= 0 && preferredIndex < buttons.length) {
        const b = buttons[preferredIndex];
        return !!(b && (b.pressed || b.value > 0.5));
      }
      return null;
    } catch (e) {
      return null;
    }
  },

  _readXrSessionButtonState: function(handedness) {
    try {
      const scene = this.el && this.el.sceneEl ? this.el.sceneEl : document.querySelector('a-scene');
      const renderer = scene && scene.renderer ? scene.renderer : null;
      const xr = renderer && renderer.xr ? renderer.xr : null;
      const session = xr && xr.getSession ? xr.getSession() : null;
      if (!session || !session.inputSources) return null;

      for (const src of session.inputSources) {
        if (!src || src.handedness !== handedness || !src.gamepad || !src.gamepad.buttons) continue;
        const buttons = src.gamepad.buttons;
        const isPressed = (b) => !!(b && (b.pressed || b.value > 0.5));
        return {
          pressed: isPressed(buttons[3]) || isPressed(buttons[4]),
          values: buttons.slice(0, 8).map((b) => (b ? Number(b.value || 0) : 0)),
        };
      }
      return null;
    } catch (e) {
      return null;
    }
  },

  tick: function () {
    // Update controller text if controllers are visible
    if (!this.leftHand || !this.rightHand) return; // Added safety check

    // Button polling path 1: A-Frame tracked-controls gamepad.
    const leftXPolled = this._readGamepadButtonPressed(this.leftHand, 4);
    const rightAPolled = this._readGamepadButtonPressed(this.rightHand, 4);
    if (leftXPolled !== null) this.leftXDown = leftXPolled;
    if (rightAPolled !== null) this.rightADown = rightAPolled;
    // Button polling path 2: XRSession inputSources (more robust on Quest browser).
    const leftXrPolled = this._readXrSessionButtonState('left');
    const rightXrPolled = this._readXrSessionButtonState('right');
    if (leftXrPolled !== null) this.leftXDown = leftXrPolled.pressed;
    if (rightXrPolled !== null) this.rightADown = rightXrPolled.pressed;

    // --- BEGIN DETAILED LOGGING ---
    if (this.leftHand.object3D) {
      // console.log(`Left Hand Raw - Visible: ${this.leftHand.object3D.visible}, Pos: ${this.leftHand.object3D.position.x.toFixed(2)},${this.leftHand.object3D.position.y.toFixed(2)},${this.leftHand.object3D.position.z.toFixed(2)}`);
    }
    if (this.rightHand.object3D) {
      // console.log(`Right Hand Raw - Visible: ${this.rightHand.object3D.visible}, Pos: ${this.rightHand.object3D.position.x.toFixed(2)},${this.rightHand.object3D.position.y.toFixed(2)},${this.rightHand.object3D.position.z.toFixed(2)}`);
    }
    // --- END DETAILED LOGGING ---

    // Collect data from both controllers
    const leftController = {
        hand: 'left',
        position: null,
        rotation: null,
        gripActive: false,
        trigger: 0,
        x: 0,
        xButton: this.leftXDown,
        buttons: leftXrPolled ? leftXrPolled.values : []
    };
    
    const rightController = {
        hand: 'right',
        position: null,
        rotation: null,
        gripActive: false,
        trigger: 0,
        a: 0,
        aButton: this.rightADown,
        buttons: rightXrPolled ? rightXrPolled.values : []
    };

    // Update Left Hand Text & Collect Data
    if (this.leftHand && this.leftHand.object3D) {
        const leftPos = this.leftHand.object3D.position;
        const leftRotEuler = this.leftHand.object3D.rotation; // Euler angles in radians
        // Convert to degrees without offset
        const leftRotX = THREE.MathUtils.radToDeg(leftRotEuler.x);
        const leftRotY = THREE.MathUtils.radToDeg(leftRotEuler.y);
        const leftRotZ = THREE.MathUtils.radToDeg(leftRotEuler.z);

        // Calculate relative rotation if grip is held
        if (this.leftGripDown && this.leftGripInitialRotation) {
          this.leftRelativeRotation = this.calculateRelativeRotation(
            { x: leftRotX, y: leftRotY, z: leftRotZ },
            this.leftGripInitialRotation
          );
          
          // Calculate Z-axis rotation using quaternions
          if (this.leftGripInitialQuaternion) {
            this.leftZAxisRotation = this.calculateZAxisRotation(
              this.leftHand.object3D.quaternion,
              this.leftGripInitialQuaternion
            );
          }
          
          console.log('Left relative rotation:', this.leftRelativeRotation);
          console.log('Left Z-axis rotation:', this.leftZAxisRotation.toFixed(1), 'degrees');
        }

        // Create display text including relative rotation when grip is held
        let combinedLeftText = `Pos: ${leftPos.x.toFixed(2)} ${leftPos.y.toFixed(2)} ${leftPos.z.toFixed(2)}\\nRot: ${leftRotX.toFixed(0)} ${leftRotY.toFixed(0)} ${leftRotZ.toFixed(0)}`;
        if (this.leftGripDown && this.leftGripInitialRotation) {
          combinedLeftText += `\\nZ-Rot: ${this.leftZAxisRotation.toFixed(1)}°`;
        }

        if (this.leftHandInfoText) {
            this.leftHandInfoText.setAttribute('value', combinedLeftText);
        }

        // Collect left controller data
        leftController.position = { x: leftPos.x, y: leftPos.y, z: leftPos.z };
        leftController.rotation = { x: leftRotX, y: leftRotY, z: leftRotZ };
        leftController.quaternion = { 
          x: this.leftHand.object3D.quaternion.x, 
          y: this.leftHand.object3D.quaternion.y, 
          z: this.leftHand.object3D.quaternion.z, 
          w: this.leftHand.object3D.quaternion.w 
        };
        leftController.trigger = this.leftTriggerDown ? 1 : 0;
        leftController.x = this.leftXDown ? 1 : 0;
        leftController.gripActive = this.leftGripDown;
        leftController.xButton = this.leftXDown;
    }

    // Update Right Hand Text & Collect Data
    if (this.rightHand && this.rightHand.object3D) {
        const rightPos = this.rightHand.object3D.position;
        const rightRotEuler = this.rightHand.object3D.rotation; // Euler angles in radians
        // Convert to degrees without offset
        const rightRotX = THREE.MathUtils.radToDeg(rightRotEuler.x);
        const rightRotY = THREE.MathUtils.radToDeg(rightRotEuler.y);
        const rightRotZ = THREE.MathUtils.radToDeg(rightRotEuler.z);

        // Calculate relative rotation if grip is held
        if (this.rightGripDown && this.rightGripInitialRotation) {
          this.rightRelativeRotation = this.calculateRelativeRotation(
            { x: rightRotX, y: rightRotY, z: rightRotZ },
            this.rightGripInitialRotation
          );
          
          // Calculate Z-axis rotation using quaternions
          if (this.rightGripInitialQuaternion) {
            this.rightZAxisRotation = this.calculateZAxisRotation(
              this.rightHand.object3D.quaternion,
              this.rightGripInitialQuaternion
            );
          }
          
          console.log('Right relative rotation:', this.rightRelativeRotation);
          console.log('Right Z-axis rotation:', this.rightZAxisRotation.toFixed(1), 'degrees');
        }

        // Create display text including relative rotation when grip is held
        let combinedRightText = `Pos: ${rightPos.x.toFixed(2)} ${rightPos.y.toFixed(2)} ${rightPos.z.toFixed(2)}\\nRot: ${rightRotX.toFixed(0)} ${rightRotY.toFixed(0)} ${rightRotZ.toFixed(0)}`;
        if (this.rightGripDown && this.rightGripInitialRotation) {
          combinedRightText += `\\nZ-Rot: ${this.rightZAxisRotation.toFixed(1)}°`;
        }

        if (this.rightHandInfoText) {
            this.rightHandInfoText.setAttribute('value', combinedRightText);
        }

        // Collect right controller data
        rightController.position = { x: rightPos.x, y: rightPos.y, z: rightPos.z };
        rightController.rotation = { x: rightRotX, y: rightRotY, z: rightRotZ };
        rightController.quaternion = { 
          x: this.rightHand.object3D.quaternion.x, 
          y: this.rightHand.object3D.quaternion.y, 
          z: this.rightHand.object3D.quaternion.z, 
          w: this.rightHand.object3D.quaternion.w 
        };
        rightController.trigger = this.rightTriggerDown ? 1 : 0;
        rightController.a = this.rightADown ? 1 : 0;
        rightController.gripActive = this.rightGripDown;
        rightController.aButton = this.rightADown;
    }

    // Send combined packet if WebSocket is open and at least one controller has valid data
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        const hasValidLeft = !!leftController.quaternion;
        const hasValidRight = !!rightController.quaternion;
        const calibrationRequested = leftController.xButton && rightController.aButton;
        
        if (hasValidLeft || hasValidRight || calibrationRequested) {
            const dualControllerData = {
                timestamp: Date.now(),
                leftController: leftController,
                rightController: rightController
            };
            this.websocket.send(JSON.stringify(dualControllerData));
        }
    }
  }
});


// Add the component to the scene after it's loaded
document.addEventListener('DOMContentLoaded', (event) => {
    const scene = document.querySelector('a-scene');

    if (scene) {
        // Listen for controller connection events
        scene.addEventListener('controllerconnected', (evt) => {
            console.log('Controller CONNECTED:', evt.detail.name, evt.detail.component.data.hand);
            debugLog(`🕹️ Controller CONNECTED: ${evt.detail.component.data.hand}`);
        });
        scene.addEventListener('controllerdisconnected', (evt) => {
            console.log('Controller DISCONNECTED:', evt.detail.name, evt.detail.component.data.hand);
            debugLog(`⚪ Controller DISCONNECTED: ${evt.detail.component.data.hand}`);
        });

        // Add controller-updater component when scene is loaded (A-Frame manages session)
        if (scene.hasLoaded) {
            scene.setAttribute('controller-updater', '');
            console.log("controller-updater component added immediately.");
        } else {
            scene.addEventListener('loaded', () => {
                scene.setAttribute('controller-updater', '');
                console.log("controller-updater component added after scene loaded.");
            });
        }
    } else {
        console.error('A-Frame scene not found!');
    }

    // Add controller tracking button logic
    addControllerTrackingButton();
});

function configureSceneForPassthrough(sceneEl) {
    if (!sceneEl) return;
    sceneEl.setAttribute('renderer', 'alpha: true; antialias: auto; colorManagement: true');
    sceneEl.setAttribute(
        'webxr',
        'referenceSpaceType: local-floor; requiredFeatures: local-floor; optionalFeatures: bounded-floor, hand-tracking, dom-overlay'
    );

    if (sceneEl.renderer) {
        sceneEl.renderer.setClearAlpha(0.0);
    }
    if (sceneEl.object3D) {
        sceneEl.object3D.background = null;
    }
}

async function enterBestXrMode(sceneEl, arSupported, vrSupported) {
    configureSceneForPassthrough(sceneEl);
    if (!sceneEl.renderer || !sceneEl.renderer.xr) {
        throw new Error('A-Frame renderer is not ready yet.');
    }

    if (arSupported && navigator.xr) {
        debugLog('🟢 Trying passthrough AR...');
        try {
            if (typeof sceneEl.enterAR === 'function') {
                await sceneEl.enterAR();
            } else if (typeof sceneEl.enterVR === 'function') {
                // A-Frame internally supports AR via enterVR(true), but enterAR is preferred.
                await sceneEl.enterVR(true);
            } else {
                throw new Error('A-Frame AR entry API not available.');
            }
            debugLog('✅ Started in immersive-ar (passthrough).');
            return 'immersive-ar';
        } catch (arErr) {
            debugLog(`⚠️ immersive-ar failed, fallback to VR: ${arErr.message}`);
        }
    }

    if (vrSupported) {
        debugLog('🟡 Starting immersive-vr...');
        if (typeof sceneEl.enterVR === 'function') {
            await sceneEl.enterVR();
        } else {
            throw new Error('A-Frame VR entry API not available.');
        }
        debugLog('✅ Started in immersive-vr.');
        return 'immersive-vr';
    }

    throw new Error('Neither immersive-ar nor immersive-vr is available on this device/browser.');
}

function addControllerTrackingButton() {
    console.log('addControllerTrackingButton called');
    if (navigator.xr) {
        console.log('WebXR is supported, checking session types...');
        // Check for either immersive-ar (Quest 3/Pro) or immersive-vr (Quest 2)
        Promise.all([
            navigator.xr.isSessionSupported('immersive-ar').catch(() => false),
            navigator.xr.isSessionSupported('immersive-vr').catch(() => false)
        ]).then(([arSupported, vrSupported]) => {
            console.log(`XR support check - AR: ${arSupported}, VR: ${vrSupported}`);
            if (arSupported || vrSupported) {
                console.log('Creating Start Controller Tracking button...');
                // Create Start Controller Tracking button
                const startButton = document.createElement('button');
                startButton.id = 'start-tracking-button';
                startButton.textContent = 'Start Controller Tracking';
                startButton.style.position = 'fixed';
                startButton.style.top = '50%';
                startButton.style.left = '50%';
                startButton.style.transform = 'translate(-50%, -50%)';
                startButton.style.padding = '20px 40px';
                startButton.style.fontSize = '20px';
                startButton.style.fontWeight = 'bold';
                startButton.style.backgroundColor = '#4CAF50';
                startButton.style.color = 'white';
                startButton.style.border = 'none';
                startButton.style.borderRadius = '8px';
                startButton.style.cursor = 'pointer';
                startButton.style.zIndex = '99999';
                startButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
                startButton.style.transition = 'all 0.3s ease';

                // Hover effects
                startButton.addEventListener('mouseenter', () => {
                    startButton.style.backgroundColor = '#45a049';
                    startButton.style.transform = 'translate(-50%, -50%) scale(1.05)';
                });
                startButton.addEventListener('mouseleave', () => {
                    startButton.style.backgroundColor = '#4CAF50';
                    startButton.style.transform = 'translate(-50%, -50%) scale(1)';
                });

                startButton.onclick = async () => {
                    debugLog('🎮 Button clicked!');
                    console.log('Start Controller Tracking button clicked.');
                    const sceneEl = document.querySelector('a-scene');
                    if (!sceneEl) {
                        console.error('A-Frame scene not found for enterVR call!');
                        alert('VR scene not loaded. Please refresh the page.');
                        debugLog('❌ Scene not found');
                        return;
                    }

                    debugLog('⏳ Checking scene loaded...');
                    // Wait for scene to be fully loaded
                    if (!sceneEl.hasLoaded) {
                        console.log('Scene not loaded yet, waiting...');
                        debugLog('⏳ Waiting for scene...');
                        await new Promise(resolve => {
                            sceneEl.addEventListener('loaded', resolve, { once: true });
                        });
                    }
                    debugLog('✅ Scene loaded');

                    // Update button to show we're connecting
                    startButton.textContent = 'Connecting...';
                    startButton.disabled = true;

                    try {
                        debugLog('🔌 Checking robot...');
                        // Check if robot is already connected
                        const statusResponse = await fetch('/api/status');
                        const status = await statusResponse.json();

                        // Only try to connect robot if not in no-robot mode
                        if (status.robotEngaged === false && status.left_arm_connected !== undefined) {
                            console.log('Robot not connected. Connecting arms first...');
                            startButton.textContent = 'Connecting Arms...';
                            debugLog('🤖 Connecting robot...');

                            // Connect the robot arms
                            const connectResponse = await fetch('/api/robot', {
                                method: 'POST',
                                headers: {
                                    'Content-Type': 'application/json'
                                },
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
                                console.log('Robot arms connected successfully.');
                                await new Promise(resolve => setTimeout(resolve, 500));
                            }
                        } else {
                            console.log('Robot already connected or in no-robot mode.');
                            debugLog('✅ Robot OK');
                        }

                        // Start XR mode: prefer passthrough AR, then fallback to VR.
                        console.log('Requesting XR session...');
                        startButton.textContent = 'Starting XR...';
                        debugLog('🥽 Entering VR...');
                        const mode = await enterBestXrMode(sceneEl, arSupported, vrSupported);
                        console.log(`XR started in mode: ${mode}`);
                        startButton.style.display = 'none';
                    } catch (err) {
                        console.error('Failed to start controller tracking:', err);
                        debugLog(`❌ Error: ${err.message}`);
                        alert(`Failed to start: ${err.message}`);
                        // Reset button state
                        startButton.textContent = 'Start Controller Tracking';
                        startButton.disabled = false;
                    }
                };

                document.body.appendChild(startButton);
                console.log('Start Controller Tracking button added to page with z-index 99999');

                // Add VR instructions panel
                createVrInstructionsPanel();

                // Show the back to desktop button (function defined in interface.js)
                if (typeof showBackToDesktopButton === 'function') {
                    showBackToDesktopButton();
                }

                // Listen for VR session events to hide/show start button
                const sceneEl = document.querySelector('a-scene');
                if (sceneEl) {
                    sceneEl.addEventListener('enter-vr', () => {
                        console.log('Entered VR - hiding start button');
                        startButton.style.display = 'none';
                    });

                    sceneEl.addEventListener('exit-vr', () => {
                        console.log('Exited VR - showing start button');
                        startButton.style.display = 'block';
                    });
                }

            } else {
                console.warn('Neither immersive-ar nor immersive-vr supported by this browser/device.');
            }
        }).catch((err) => {
            console.error('Error checking XR support:', err);
        });
    } else {
        console.warn('WebXR not supported by this browser.');
    }
}

function createVrInstructionsPanel() {
    // Don't create if already exists
    if (document.getElementById('vr-instructions-panel')) return;

    const panel = document.createElement('div');
    panel.id = 'vr-instructions-panel';
    panel.style.cssText = `
        position: fixed;
        top: 20px;
        left: 50%;
        transform: translateX(-50%);
        max-width: 90%;
        width: 600px;
        background: rgba(15, 52, 96, 0.95);
        border-radius: 12px;
        padding: 20px;
        color: white;
        z-index: 9998;
        box-shadow: 0 8px 32px rgba(0,0,0,0.3);
        border: 1px solid rgba(255,255,255,0.1);
    `;

    panel.innerHTML = `
        <h2 style="margin: 0 0 15px 0; font-size: 1.2em; text-align: center;">VR Controller Instructions</h2>
        <div style="display: flex; gap: 15px; align-items: flex-start; flex-wrap: wrap;">
            <div style="flex: 1; min-width: 150px; text-align: center;">
                <img src="media/telegrip_instructions.jpg" alt="VR Controller Instructions"
                     style="max-width: 100%; height: auto; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.2);">
            </div>
            <div style="flex: 1; min-width: 200px; display: flex; flex-direction: column; gap: 8px; font-size: 14px;">
                <div style="padding: 8px; background: rgba(255,255,255,0.1); border-radius: 6px;">
                    <strong style="color: #ee4d9a;">Grip Button:</strong> Hold to move the arm
                </div>
                <div style="padding: 8px; background: rgba(255,255,255,0.1); border-radius: 6px;">
                    <strong style="color: #9af58c;">Trigger:</strong> Hold to close gripper
                </div>
            </div>
        </div>
    `;

    document.body.appendChild(panel);

    // Hide panel when entering VR
    const sceneEl = document.querySelector('a-scene');
    if (sceneEl) {
        sceneEl.addEventListener('enter-vr', () => {
            panel.style.display = 'none';
        });
        sceneEl.addEventListener('exit-vr', () => {
            panel.style.display = 'block';
        });
    }
} 
