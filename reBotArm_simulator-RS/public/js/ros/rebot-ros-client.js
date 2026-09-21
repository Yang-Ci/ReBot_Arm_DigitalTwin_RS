(function () {
  class ReBotRosClient extends EventTarget {
    constructor(options) {
      super();
      this.url = options && options.url ? options.url : '';
      this.namespace = options && options.namespace ? options.namespace : 'rebotarm';
      this.socket = null;
      this.connected = false;
      this.autoReconnect = true;
      this.reconnectDelay = 1400;
      this._subscriptions = new Map();
      this._advertisedTopics = new Set();
      this._pendingServices = new Map();
      this._pendingActions = new Map();
      this._nextId = 1;
      this._manualClose = false;
      this._lastMessageAt = new Map();
      this._connectSeq = 0;
    }

    connect(url) {
      if (url) this.url = url;
      this._manualClose = false;
      if (this.socket && this.socket.readyState === WebSocket.OPEN) {
        this._emitStatus('open', 'ROS 已连接');
        return;
      }
      if (this.socket && this.socket.readyState === WebSocket.CONNECTING) return;

      const seq = ++this._connectSeq;
      this._emitStatus('connecting', `正在连接 ${this.url}`);
      this.socket = new WebSocket(this.url);
      const socket = this.socket;

      socket.addEventListener('open', () => {
        if (seq !== this._connectSeq || socket !== this.socket) return;
        this.connected = true;
        this._emitStatus('open', 'ROS 已连接');
        this._resubscribe();
      });

      socket.addEventListener('message', (event) => {
        if (seq === this._connectSeq && socket === this.socket) this._handleMessage(event);
      });
      socket.addEventListener('error', () => {
        if (seq === this._connectSeq && socket === this.socket) this._emitStatus('error', 'ROS WebSocket 出错');
      });
      socket.addEventListener('close', () => {
        if (seq !== this._connectSeq || socket !== this.socket) return;
        this.connected = false;
        this._rejectPendingServices('ROS 连接已断开');
        this._rejectPendingActions('ROS 连接已断开');
        this._emitStatus('closed', 'ROS 已断开');
        if (!this._manualClose && this.autoReconnect) {
          window.setTimeout(() => this.connect(), this.reconnectDelay);
        }
     });
   }

 disconnect() {
   this._manualClose = true;
   this.autoReconnect = false;
    this.connected = false;
    this._rejectPendingServices('ROS 连接已断开');
    this._rejectPendingActions('ROS 连接已断开');
   if (this.socket) this.socket.close();
   this.socket = null;
    this._emitStatus('closed', 'ROS 已断开');
 }

    subscribe(topic, type, callback, options) {
      const throttleRate = options && options.throttleRate ? options.throttleRate : 80;
      this._subscriptions.set(topic, { topic, type, callback, throttleRate });
      if (this.connected) this._sendSubscribe(topic, type, throttleRate);
    }

    unsubscribe(topic) {
      this._subscriptions.delete(topic);
      this._send({ op: 'unsubscribe', topic });
    }

    callService(service, type, args, options) {
      const retryOnTimeout = !(options && options.retryOnTimeout === false);
      return this._callServiceOnce(service, type, args, options).catch((error) => {
        const msg = error.message || '';
       // DDS discovery delay: first service call may time out because the
       // service proxy hasn't been discovered yet.  Retry once after a brief
       // wait — by then discovery has completed and the retry succeeds.
        if (retryOnTimeout && /服务超时|timed out|timeout|No service|not available/i.test(msg)) {
          return new Promise((resolve) => setTimeout(resolve, 1500))
            .then(() => this._callServiceOnce(service, type, args, options));
        }
        throw error;
      });
    }

    _callServiceOnce(service, type, args, options) {
      const id = this._id('service');
      const timeoutMs = Number(options && options.timeoutMs) || 10000;
      return new Promise((resolve, reject) => {
        if (!this.connected) {
          reject(new Error('ROS 未连接'));
          return;
        }
        const timer = setTimeout(() => {
          if (this._pendingServices.has(id)) {
            this._pendingServices.delete(id);
            reject(new Error(`服务超时（${timeoutMs / 1000}秒）：${service}`));
          }
        }, timeoutMs);
        this._pendingServices.set(id, {
          resolve: (value) => { clearTimeout(timer); resolve(value); },
          reject: (error) => { clearTimeout(timer); reject(error); }
        });
        this._send({
          op: 'call_service',
          id,
          service,
          type,
          args: args || {},
          // rosbridge has its own (normally 5 s) service timeout. Keep it in
          // sync with the browser-side timer so a valid slow IK response is
          // not discarded halfway through the request.
          timeout: Math.max(timeoutMs / 1000, 0.1)
        });
      });
    }

    enable() {
      return this.callService(`/${this.namespace}/enable`, 'std_srvs/srv/Trigger', {});
    }

    disable() {
      return this.callService(`/${this.namespace}/disable`, 'std_srvs/srv/Trigger', {}, {
        timeoutMs: 45000,
        retryOnTimeout: false
      });
    }

    safeHome() {
      return this.callService(`/${this.namespace}/safe_home`, 'std_srvs/srv/Trigger', {}, {
        timeoutMs: 45000,
        retryOnTimeout: false
      });
    }

    startGravityCompensation() {
      return this.callService(`/${this.namespace}/gravity_compensation/start`, 'std_srvs/srv/Trigger', {});
    }

    stopGravityCompensation() {
      return this.callService(`/${this.namespace}/gravity_compensation/stop`, 'std_srvs/srv/Trigger', {});
    }

    gravityCompensationStatus() {
      return this.callService(`/${this.namespace}/gravity_compensation/status`, 'std_srvs/srv/Trigger', {});
    }

    setGripper(position, maxEffort) {
      return this.callService(`/${this.namespace}/gripper/set`, 'rebotarm_msgs/srv/SetGripper', {
        position,
        max_effort: maxEffort || 0
      });
    }

    releaseGripper() {
      return this.callService(`/${this.namespace}/gripper/release`, 'std_srvs/srv/Trigger', {});
    }

    holdGripper() {
      return this.callService(`/${this.namespace}/gripper/hold`, 'std_srvs/srv/Trigger', {});
    }

    startGripperAssist() {
      return this.callService(`/${this.namespace}/gripper/assist/start`, 'std_srvs/srv/Trigger', {});
    }

    gripperAssistStatus() {
      return this.callService(`/${this.namespace}/gripper/assist/status`, 'std_srvs/srv/Trigger', {});
    }

    moveToPose(pose, duration) {
      return this.sendActionGoal(`/${this.namespace}/move_to_pose`, 'rebotarm_msgs/action/MoveToPose', {
        target_pose: pose,
        duration: Number(duration) || 2
      }, {
        timeoutMs: Math.max(30000, ((Number(duration) || 2) + 10) * 1000)
      });
    }

    solveMoveToPoseIK(pose) {
      return this.callService(`/${this.namespace}/move_to_pose_ik`, 'rebotarm_msgs/srv/MoveToPoseIK', {
        target_pose: pose
      }, {
        timeoutMs: 20000,
        retryOnTimeout: false
      });
    }

    followJointTrajectory(jointNames, points, options) {
      const expectedDuration = this._estimateTrajectoryDuration(points);
      const frameId = options && options.profile === 'teaching-replay'
        ? 'rebotarm_teaching_replay'
        : '';
      return this.sendActionGoal(`/${this.namespace}/follow_joint_trajectory`, 'control_msgs/action/FollowJointTrajectory', {
        trajectory: {
          header: { stamp: { sec: 0, nanosec: 0 }, frame_id: frameId },
          joint_names: jointNames,
          points
        },
        goal_tolerance: [],
        path_tolerance: [],
        goal_time_tolerance: { sec: 0, nanosec: 0 }
      }, {
        timeoutMs: Math.max(30000, (expectedDuration + 10) * 1000)
      });
    }

    sendActionGoal(actionName, actionType, goal, options) {
      const id = this._id('action');
      const timeoutMs = Number(options && options.timeoutMs) || 30000;
      let retried = false;
      return new Promise((resolve, reject) => {
        if (!this.connected) {
          reject(new Error('ROS 未连接'));
          return;
        }
        const timer = setTimeout(() => {
          if (this._pendingActions.has(id)) {
            this._pendingActions.delete(id);
            reject(new Error(`动作超时（${timeoutMs / 1000}秒）：${actionName}`));
          }
        }, timeoutMs);
        this._pendingActions.set(id, {
          resolve: (value) => { clearTimeout(timer); resolve(value); },
          reject: (error) => {
            clearTimeout(timer);
            if (!retried && /No action server available/i.test(error.message || '')) {
              retried = true;
              setTimeout(() => {
                this.sendActionGoal(actionName, actionType, goal, options)
                  .then(resolve, reject);
              }, 1500);
            } else {
              reject(error);
            }
          },
          action: actionName
        });
        this._send({
          op: 'send_action_goal',
          id,
          action: actionName,
          action_type: actionType,
          args: goal || {},
          feedback: true
        });
      });
    }

    _estimateTrajectoryDuration(points) {
      const list = Array.isArray(points) ? points : [];
      let elapsed = 0;
      let previous = null;
      let previousTime = 0;
      list.forEach((point) => {
        const stamp = point && point.time_from_start ? point.time_from_start : {};
        const requestedTime = (Number(stamp.sec) || 0) + (Number(stamp.nanosec) || 0) * 1e-9;
        const positions = Array.isArray(point && point.positions)
          ? point.positions.map(Number)
          : [];
        const requestedSegment = Math.max(0, requestedTime - previousTime);
        let velocityLimitedSegment = 0;
        if (previous && previous.length === positions.length) {
          const maxDelta = positions.reduce((largest, value, index) => {
            const delta = Math.abs(value - previous[index]);
            return Number.isFinite(delta) ? Math.max(largest, delta) : largest;
          }, 0);
          velocityLimitedSegment = maxDelta / 0.6;
        }
        elapsed += Math.max(requestedSegment, velocityLimitedSegment);
        previous = positions;
        previousTime = requestedTime;
      });
      return Math.max(elapsed, previousTime, 0);
    }

    getRosTopics() {
      return this.callService('/rosapi/topics', 'rosapi_msgs/srv/Topics', {}, {
        timeoutMs: 8000,
        retryOnTimeout: false
      });
    }

    getRosServices() {
      return this.callService('/rosapi/services', 'rosapi_msgs/srv/Services', {}, {
        timeoutMs: 8000,
        retryOnTimeout: false
      });
    }

    getLastMessageAt(topic) {
      return this._lastMessageAt.get(topic) || 0;
    }

    publishJointCommand(jointName, position, options) {
      const topic = `/${this.namespace}/joints/${jointName}/cmd/mit`;
      const type = 'rebotarm_msgs/msg/JointMitCmd';
      this.advertise(topic, type);
      this.publish(topic, {
        pos: position,
        // For RS web commands, vel is treated as the maximum target slew
        // rate by the controller while the periodic MIT holding loop stays on.
        vel: options && typeof options.vlim === 'number' ? options.vlim : 1.20,
        kp: 0,
        kd: 0,
        tau: 0,
        stamp: { sec: 0, nanosec: 0 }
      });
    }

    publishGripperCommand(position, vlim) {
      const topic = `/${this.namespace}/gripper/cmd/mit`;
      const type = 'rebotarm_msgs/msg/JointMitCmd';
      this.advertise(topic, type);
      this.publish(topic, {
        pos: position,
        vel: typeof vlim === 'number' ? vlim : 5.0,
        kp: 0,
        kd: 0,
        tau: 0,
        stamp: { sec: 0, nanosec: 0 }
      });
    }

    publishTargetPose(pose) {
      const topic = `/${this.namespace}/mujoco/target_pose`;
      const type = 'geometry_msgs/msg/PoseStamped';
      this.advertise(topic, type);
      this.publish(topic, {
        header: {
          stamp: { sec: 0, nanosec: 0 },
          frame_id: 'base_link'
        },
        pose: {
          position: {
            x: Number(pose && pose.position ? pose.position.x : pose && pose.x) || 0,
            y: Number(pose && pose.position ? pose.position.y : pose && pose.y) || 0,
            z: Number(pose && pose.position ? pose.position.z : pose && pose.z) || 0
          },
          orientation: pose && pose.orientation ? pose.orientation : { x: 0, y: 0, z: 0, w: 1 }
        }
      });
    }

    advertise(topic, type) {
      if (this._advertisedTopics.has(topic)) return;
      this._advertisedTopics.add(topic);
      this._send({ op: 'advertise', topic, type });
    }

    publish(topic, msg) {
      this._send({ op: 'publish', topic, msg });
    }

    _resubscribe() {
      this._subscriptions.forEach((sub) => {
        this._sendSubscribe(sub.topic, sub.type, sub.throttleRate);
      });
    }

    _sendSubscribe(topic, type, throttleRate) {
      this._send({
        op: 'subscribe',
        id: this._id('sub'),
        topic,
        type,
        throttle_rate: throttleRate
      });
    }

    _handleMessage(event) {
      let data;
      try {
        data = JSON.parse(event.data);
      } catch (error) {
        this._emitStatus('error', '收到无法解析的 ROS 消息');
        return;
      }

      if (data.op === 'publish') {
        this._lastMessageAt.set(data.topic, Date.now());
        const sub = this._subscriptions.get(data.topic);
        if (sub) sub.callback(data.msg, data.topic);
        return;
      }

      if (data.op === 'service_response') {
        const pending = this._pendingServices.get(data.id);
        if (!pending) return;
        this._pendingServices.delete(data.id);
        if (data.result === false) {
          pending.reject(new Error(data.values && data.values.message ? data.values.message : 'ROS service failed'));
        } else {
          pending.resolve(data.values || {});
        }
        return;
      }

      if (data.op === 'action_feedback') {
        const pending = this._pendingActions.get(data.id);
        if (!pending) return;
        this.dispatchEvent(new CustomEvent('action-feedback', {
          detail: {
            id: data.id,
            action: data.action || pending.action,
            values: data.values || {}
          }
        }));
        return;
      }

      if (data.op === 'action_result') {
        const pending = this._pendingActions.get(data.id);
        if (!pending) return;
        this._pendingActions.delete(data.id);
        if (data.result === false) {
          const message = typeof data.values === 'string'
            ? data.values
            : (data.values && data.values.message ? data.values.message : 'ROS action failed');
          pending.reject(new Error(message));
          return;
        }
        pending.resolve({
          ...(data.values || {}),
          status: data.status,
          action: data.action || pending.action,
          completed: true
        });
      }
    }

    _send(payload) {
      if (!this.socket || this.socket.readyState !== WebSocket.OPEN) return;
      this.socket.send(JSON.stringify(payload));
    }

    _rejectPendingServices(message) {
      this._pendingServices.forEach((pending) => pending.reject(new Error(message)));
      this._pendingServices.clear();
    }

    _rejectPendingActions(message) {
      this._pendingActions.forEach((pending) => pending.reject(new Error(message)));
      this._pendingActions.clear();
    }

    _id(prefix) {
      const id = `${prefix}:${this._nextId}`;
      this._nextId += 1;
      return id;
    }

    _uuid() {
      const values = new Uint8Array(16);
      if (window.crypto && typeof window.crypto.getRandomValues === 'function') {
        window.crypto.getRandomValues(values);
      } else {
        for (let i = 0; i < values.length; i += 1) {
          values[i] = Math.floor(Math.random() * 256);
        }
      }
      return Array.from(values);
    }

    _emitStatus(state, message) {
      this.dispatchEvent(new CustomEvent('status', { detail: { state, message } }));
    }
  }

  window.ReBotRosClient = ReBotRosClient;
})();
