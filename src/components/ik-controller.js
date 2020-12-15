import { THREE } from "aframe";
import { waitForDOMContentLoaded } from "../utils/async-utils";
const { Vector3, Quaternion, Matrix4, Euler } = THREE;

function quaternionAlmostEquals(epsilon, u, v) {
  // Note: q and -q represent same rotation
  return (
    (Math.abs(u.x - v.x) < epsilon &&
      Math.abs(u.y - v.y) < epsilon &&
      Math.abs(u.z - v.z) < epsilon &&
      Math.abs(u.w - v.w) < epsilon) ||
    (Math.abs(-u.x - v.x) < epsilon &&
      Math.abs(-u.y - v.y) < epsilon &&
      Math.abs(-u.z - v.z) < epsilon &&
      Math.abs(-u.w - v.w) < epsilon)
  );
}

/**
 * Provides access to the end effectors for IK.
 * @namespace avatar
 * @component ik-root
 */
AFRAME.registerComponent("ik-root", {
  schema: {
    camera: { type: "string", default: ".camera" },
    leftController: { type: "string", default: ".left-controller" },
    rightController: { type: "string", default: ".right-controller" }
  },
  update(oldData) {
    if (this.data.camera !== oldData.camera) {
      this.camera = this.el.querySelector(this.data.camera);
    }

    if (this.data.leftController !== oldData.leftController) {
      this.leftController = this.el.querySelector(this.data.leftController);
    }

    if (this.data.rightController !== oldData.rightController) {
      this.rightController = this.el.querySelector(this.data.rightController);
    }
  }
});

function findIKRoot(entity) {
  while (entity && !(entity.components && entity.components["ik-root"])) {
    entity = entity.parentNode;
  }
  return entity && entity.components["ik-root"];
}

const HAND_ROTATIONS = {
  left: new Matrix4().makeRotationFromEuler(new Euler(-Math.PI / 2, Math.PI / 2, 0)),
  right: new Matrix4().makeRotationFromEuler(new Euler(-Math.PI / 2, -Math.PI / 2, 0))
};

// const angleOnXZPlaneBetweenMatrixRotations = (function() {
//   const XZ_PLANE_NORMAL = new THREE.Vector3(0, -1, 0);
//   const v1 = new THREE.Vector3();
//   const v2 = new THREE.Vector3();
//   return function angleOnXZPlaneBetweenMatrixRotations(matrixA, matrixB) {
//     v1.setFromMatrixColumn(matrixA, 2).projectOnPlane(XZ_PLANE_NORMAL);
//     v2.setFromMatrixColumn(matrixB, 2).projectOnPlane(XZ_PLANE_NORMAL);
//     return v1.angleTo(v2);
//   };
// })();

/**
 * Performs IK on a hip-rooted skeleton to align the hip, head and hands with camera and controller inputs.
 * @namespace avatar
 * @component ik-controller
 */
AFRAME.registerComponent("ik-controller", {
  schema: {
    leftEye: { type: "string", default: "LeftEye" },
    rightEye: { type: "string", default: "RightEye" },
    head: { type: "string", default: "Head" },
    neck: { type: "string", default: "Neck" },
    leftHand: { type: "string", default: "LeftHand" },
    rightHand: { type: "string", default: "RightHand" },
    // vrdevel modified start
    // chest: { type: "string", default: "Spine" },
    hips: { type: "string", default: "Hips" },
    chest: { type: "string", default: "Chest" },
    spine: { type: "string", default: "Spine" },
    leftUpLeg: { type: "string", default: "LeftUpLeg" },
    leftLeg: { type: "string", default: "LeftLeg" },
    leftFoot: { type: "string", default: "LeftFoot" },
    rightUpLeg: { type: "string", default: "RightUpLeg" },
    rightLeg: { type: "string", default: "RightLeg" },
    rightFoot: { type: "string", default: "RightFoot" },
    leftShoulder: { type: "string", default: "LeftShoulder" },
    leftArm: { type: "string", default: "LeftArm" },
    leftForeArm: { type: "string", default: "LeftForeArm" },
    rightShoulder: { type: "string", default: "RightShoulder" },
    rightArm: { type: "string", default: "RightArm" },
    rightForeArm: { type: "string", default: "RightForeArm" },
    // vrdevel modified end
    rotationSpeed: { default: 8 },
    maxLerpAngle: { default: 90 * THREE.Math.DEG2RAD },
    alwaysUpdate: { type: "boolean", default: false }
  },

  init() {
    this._runScheduledWork = this._runScheduledWork.bind(this);
    this._updateIsInView = this._updateIsInView.bind(this);

    this.flipY = new Matrix4().makeRotationY(Math.PI);

    this.cameraForward = new Matrix4();
    this.headTransform = new Matrix4();
    this.hipsPosition = new Vector3();

    this.invHipsToHeadVector = new Vector3();

    this.middleEyeMatrix = new Matrix4();
    this.middleEyePosition = new Vector3();
    this.invMiddleEyeToHead = new Matrix4();

    this.cameraYRotation = new Euler();
    this.cameraYQuaternion = new Quaternion();

    this.invHipsQuaternion = new Quaternion();
    this.headQuaternion = new Quaternion();

    this.rootToChest = new Matrix4();
    this.invRootToChest = new Matrix4();

    this.ikRoot = findIKRoot(this.el);

    this.isInView = true;
    this.hasConvergedHips = false;
    this.lastCameraTransform = new THREE.Matrix4();
    waitForDOMContentLoaded().then(() => {
      this.playerCamera = document.getElementById("viewing-camera").getObject3D("camera");
    });

    this.el.sceneEl.systems["frame-scheduler"].schedule(this._runScheduledWork, "ik");
    this.forceIkUpdate = true;
  },

  remove() {
    this.el.sceneEl.systems["frame-scheduler"].unschedule(this._runScheduledWork, "ik");
  },

  update(oldData) {
    this.avatar = this.el.object3D;
    // vrdevel modified start
    this.avatar.rotateY(Math.PI);
    // vrdevel modified end

    if (this.data.leftEye !== oldData.leftEye) {
      this.leftEye = this.el.object3D.getObjectByName(this.data.leftEye);
    }

    if (this.data.rightEye !== oldData.rightEye) {
      this.rightEye = this.el.object3D.getObjectByName(this.data.rightEye);
    }

    if (this.data.head !== oldData.head) {
      this.head = this.el.object3D.getObjectByName(this.data.head);
    }

    if (this.data.neck !== oldData.neck) {
      this.neck = this.el.object3D.getObjectByName(this.data.neck);
    }

    if (this.data.leftHand !== oldData.leftHand) {
      this.leftHand = this.el.object3D.getObjectByName(this.data.leftHand);
    }

    if (this.data.rightHand !== oldData.rightHand) {
      this.rightHand = this.el.object3D.getObjectByName(this.data.rightHand);
    }

    if (this.data.chest !== oldData.chest) {
      this.chest = this.el.object3D.getObjectByName(this.data.chest);
    }

    // vrdevel modified start
    if (this.data.hips !== oldData.hips) {
      this.hips = this.el.object3D.getObjectByName(this.data.hips);
    }

    if (this.data.spine !== oldData.spine) {
      this.spine = this.el.object3D.getObjectByName(this.data.spine);
    }

    if (this.data.leftUpLeg !== oldData.leftUpLeg) {
      this.leftUpLeg = this.el.object3D.getObjectByName(this.data.leftUpLeg);
    }

    if (this.data.leftLeg !== oldData.leftLeg) {
      this.leftLeg = this.el.object3D.getObjectByName(this.data.leftLeg);
    }

    if (this.data.leftFoot !== oldData.leftFoot) {
      this.leftFoot = this.el.object3D.getObjectByName(this.data.leftFoot);
    }

    if (this.data.rightUpLeg !== oldData.rightUpLeg) {
      this.rightUpLeg = this.el.object3D.getObjectByName(this.data.rightUpLeg);
    }

    if (this.data.rightLeg !== oldData.rightLeg) {
      this.rightLeg = this.el.object3D.getObjectByName(this.data.rightLeg);
    }

    if (this.data.rightFoot !== oldData.rightFoot) {
      this.rightFoot = this.el.object3D.getObjectByName(this.data.rightFoot);
    }

    if (this.data.leftShoulder !== oldData.leftShoulder) {
      this.leftShoulder = this.el.object3D.getObjectByName(this.data.leftShoulder);
    }

    if (this.data.leftArm !== oldData.leftArm) {
      this.leftArm = this.el.object3D.getObjectByName(this.data.leftArm);
    }

    if (this.data.leftForeArm !== oldData.leftForeArm) {
      this.leftForeArm = this.el.object3D.getObjectByName(this.data.leftForeArm);
    }

    if (this.data.rightShoulder !== oldData.rightShoulder) {
      this.rightShoulder = this.el.object3D.getObjectByName(this.data.rightShoulder);
    }

    if (this.data.rightArm !== oldData.rightArm) {
      this.rightArm = this.el.object3D.getObjectByName(this.data.rightArm);
    }

    if (this.data.rightForeArm !== oldData.rightForeArm) {
      this.rightForeArm = this.el.object3D.getObjectByName(this.data.rightForeArm);
    }

    if (this.leftShoulder && this.leftArm && this.leftForeArm && this.leftHand) {
      this.leftHandToForeArm = this.leftHand.getWorldPosition().distanceTo(this.leftForeArm.getWorldPosition());
      this.leftForeArmToArm = this.leftForeArm.getWorldPosition().distanceTo(this.leftArm.getWorldPosition());
      this.leftArmToShoulder = this.leftArm.getWorldPosition().distanceTo(this.leftShoulder.getWorldPosition());
      this.leftArmLength = this.leftHandToForeArm + this.leftForeArmToArm + this.leftArmToShoulder;
    }

    if (this.rightShoulder && this.rightArm && this.rightForeArm && this.rightHand) {
      this.rightHandToForeArm = this.rightHand.getWorldPosition().distanceTo(this.rightForeArm.getWorldPosition());
      this.rightForeArmToArm = this.rightForeArm.getWorldPosition().distanceTo(this.rightArm.getWorldPosition());
      this.rightArmToShoulder = this.rightArm.getWorldPosition().distanceTo(this.rightShoulder.getWorldPosition());
      this.rightArmLength = this.rightHandToForeArm + this.rightForeArmToArm + this.rightArmToShoulder;
    }
    // vrdevel modified end

    // Set middleEye's position to be right in the middle of the left and right eyes.
    this.middleEyePosition.addVectors(this.leftEye.position, this.rightEye.position);
    this.middleEyePosition.divideScalar(2);
    this.middleEyeMatrix.makeTranslation(this.middleEyePosition.x, this.middleEyePosition.y, this.middleEyePosition.z);
    this.invMiddleEyeToHead = this.middleEyeMatrix.getInverse(this.middleEyeMatrix);

    // vrdevel modified start
    // this.invHipsToHeadVector
    //   .addVectors(this.chest.position, this.neck.position)
    //   .add(this.head.position)
    //   .negate();
    if (this.chest) {
      this.invHipsToHeadVector
        .addVectors(this.spine.position, this.neck.position)
        .add(this.chest.position)
        .add(this.head.position)
        .negate();
    } else {
      this.invHipsToHeadVector
        .addVectors(this.spine.position, this.neck.position)
        .add(this.head.position)
        .negate();
    }
  },

  tick() {
    if (!this.ikRoot) {
      return;
    }

    const root = this.ikRoot.el.object3D;
    root.updateMatrices();
    const { camera, leftController, rightController } = this.ikRoot;

    camera.object3D.updateMatrix();

    const hasNewCameraTransform = !this.lastCameraTransform.equals(camera.object3D.matrix);

    // Optimization: if the camera hasn't moved and the hips converged to the target orientation on a previous frame,
    // then the avatar does not need any IK this frame.
    //
    // Update in-view avatars every frame, and update out-of-view avatars via frame scheduler.
    if (
      this.data.alwaysUpdate ||
      this.forceIkUpdate ||
      (this.isInView && (hasNewCameraTransform || !this.hasConvergedHips))
    ) {
      if (hasNewCameraTransform) {
        this.lastCameraTransform.copy(camera.object3D.matrix);
      }

      const {
        avatar,
        head,
        neck,
        // vrdevel modified start
        // chest,
        spine,
        // vrdevel modified end
        cameraForward,
        headTransform,
        invMiddleEyeToHead,
        invHipsToHeadVector,
        flipY,
        cameraYRotation,
        cameraYQuaternion,
        invHipsQuaternion,
        rootToChest,
        invRootToChest
      } = this;

      // Camera faces the -Z direction. Flip it along the Y axis so that it is +Z.
      cameraForward.multiplyMatrices(camera.object3D.matrix, flipY);

      // Compute the head position such that the hmd position would be in line with the middleEye
      headTransform.multiplyMatrices(cameraForward, invMiddleEyeToHead);

      // Then position the avatar such that the head is aligned with headTransform
      // (which positions middleEye in line with the hmd)
      //
      // Note that we position the avatar itself, *not* the hips, since positioning the
      // hips will use vertex skinning to do the root displacement, which results in
      // frustum culling errors since three.js does not take into account skinning when
      // computing frustum culling sphere bounds.
      avatar.position.setFromMatrixPosition(headTransform).add(invHipsToHeadVector);
      avatar.matrixNeedsUpdate = true;

      // Animate the hip rotation to follow the Y rotation of the camera with some damping.
      cameraYRotation.setFromRotationMatrix(cameraForward, "YXZ");
      cameraYRotation.x = 0;
      cameraYRotation.z = 0;
      cameraYQuaternion.setFromEuler(cameraYRotation);

      // vrdevel modified start
      // if (this._hadFirstTick) {
      //   camera.object3D.updateMatrices();
      //   avatar.updateMatrices();
      //   // Note: Camera faces down -Z, avatar faces down +Z
      //   const yDelta = Math.PI - angleOnXZPlaneBetweenMatrixRotations(camera.object3D.matrixWorld, avatar.matrixWorld);

      //   if (yDelta > this.data.maxLerpAngle) {
      //     avatar.quaternion.copy(cameraYQuaternion);
      //   } else {
      //     Quaternion.slerp(
      //       avatar.quaternion,
      //       cameraYQuaternion,
      //       avatar.quaternion,
      //       (this.data.rotationSpeed * dt) / 1000
      //     );
      //   }
      // } else {
      //   avatar.quaternion.copy(cameraYQuaternion);
      // }
      // vrdevel modified end

      this.hasConvergedHips = quaternionAlmostEquals(0.0001, cameraYQuaternion, avatar.quaternion);

      // Take the head orientation computed from the hmd, remove the Y rotation already applied to it by the hips,
      // and apply it to the head
      invHipsQuaternion.copy(avatar.quaternion).inverse();
      // vrdevel modified start
      // head.quaternion.setFromRotationMatrix(headTransform).premultiply(invHipsQuaternion);
      neck.quaternion.setFromRotationMatrix(headTransform).premultiply(invHipsQuaternion);
      // vrdevel modified end

      avatar.updateMatrix();
      // vrdevel modified start
      // rootToChest.multiplyMatrices(avatar.matrix, chest.matrix)
      rootToChest.multiplyMatrices(avatar.matrix, spine.matrix);
      if (this.chest) {
        rootToChest.multiply(this.chest.matrix);
      }
      // vrdevel modified end
      invRootToChest.getInverse(rootToChest);

      root.matrixNeedsUpdate = true;
      neck.matrixNeedsUpdate = true;
      head.matrixNeedsUpdate = true;
      // vrdevel modified start
      spine.matrixNeedsUpdate = true;
      // vrdevel modified end
    }

    // vrdevel modified start
    // const { leftHand, rightHand } = this;
    const { leftHand, rightHand, leftShoulder, leftArm, leftForeArm, rightShoulder, rightArm, rightForeArm } = this;

    // if (leftHand) this.updateHand(HAND_ROTATIONS.left, leftHand, leftController.object3D, true, this.isInView);
    // if (rightHand) this.updateHand(HAND_ROTATIONS.right, rightHand, rightController.object3D, false, this.isInView);
    if (leftHand && leftShoulder && leftArm && leftForeArm) {
      this.updateArm(
        HAND_ROTATIONS.left,
        leftHand,
        leftController.object3D,
        true,
        this.isInView,
        leftShoulder,
        leftArm,
        leftForeArm
      );
      // this.updateIk(
      //   HAND_ROTATIONS.left,
      //   leftHand,
      //   leftController.object3D,
      //   true,
      //   this.isInView,
      //   leftShoulder,
      //   leftArm,
      //   leftForeArm
      // );
    } else if (leftHand) {
      this.updateHand(HAND_ROTATIONS.left, leftHand, leftController.object3D, true, this.isInView);
    }
    if (rightHand && rightShoulder && rightArm && rightForeArm) {
      this.updateArm(
        HAND_ROTATIONS.right,
        rightHand,
        rightController.object3D,
        false,
        this.isInView,
        rightShoulder,
        rightArm,
        rightForeArm
      );
      // this.updateIk(
      //   HAND_ROTATIONS.right,
      //   rightHand,
      //   rightController.object3D,
      //   false,
      //   this.isInView,
      //   rightShoulder,
      //   rightArm,
      //   rightForeArm
      // );
    } else if (rightHand) {
      this.updateHand(HAND_ROTATIONS.right, rightHand, rightController.object3D, false, this.isInView);
    }

    // vrdevel modified end
    this.forceIkUpdate = false;

    if (!this._hadFirstTick) {
      // Ensure the avatar is not shown until we've done our first IK step, to prevent seeing mis-oriented/t-pose pose or our own avatar at the wrong place.
      this.ikRoot.el.object3D.visible = true;
      this._hadFirstTick = true;
    }
  },

  updateHand(handRotation, handObject3D, controllerObject3D, isLeft, isInView) {
    const handMatrix = handObject3D.matrix;

    // TODO: This coupling with personal-space-invader is not ideal.
    // There should be some intermediate thing managing multiple opinions about object visibility
    const spaceInvader = handObject3D.el.components["personal-space-invader"];

    if (spaceInvader) {
      // If this hand has an invader, defer to it to manage visibility overall but tell it to hide based upon controller state
      spaceInvader.setAlwaysHidden(!controllerObject3D.visible);
    } else {
      handObject3D.visible = controllerObject3D.visible;
    }

    // Optimization: skip IK update if not in view and not forced by frame scheduler
    if (controllerObject3D.visible && (isInView || this.forceIkUpdate || this.data.alwaysUpdate)) {
      handMatrix.multiplyMatrices(this.invRootToChest, controllerObject3D.matrix);

      handMatrix.multiply(handRotation);

      handObject3D.position.setFromMatrixPosition(handMatrix);
      handObject3D.rotation.setFromRotationMatrix(handMatrix);
      handObject3D.matrixNeedsUpdate = true;
    }
  },
  // vrdevel modified start
  updateIk(
    handRotation,
    handObject3D,
    controllerObject3D,
    isLeft,
    isInView,
    shoulderObject3D,
    armObject3D,
    foreArmObject3D
  ) {
    const handPosition = new THREE.Vector3();
    handObject3D.getWorldPosition(handPosition);
    const foreArmPosition = new THREE.Vector3();
    foreArmObject3D.getWorldPosition(foreArmPosition);
    const armPosition = new THREE.Vector3();
    armObject3D.getWorldPosition(armPosition);
    const shoulderPosition = new THREE.Vector3();
    shoulderObject3D.getWorldPosition(shoulderPosition);
    const controllerPosition = new THREE.Vector3();
    controllerObject3D.getWorldPosition(controllerPosition);
    if (isLeft) {
      const distance = armPosition.distanceTo(controllerPosition);
      let middleTarget = new THREE.Vector3();
      if (distance >= this.leftArmLength) {
        middleTarget = controllerPosition;
      } else {
        const forward = new THREE.Vector3();
        controllerObject3D.getWorldDirection(forward);
        forward.normalize();
        forward.multiplyScalar(-this.leftHandToForeArm);
        middleTarget.subVectors(controllerPosition, forward);

        // const belowVector = new THREE.Vector3(0, -1, 0);
        // belowVector.multiplyScalar(this.leftForeArmToArm);
        // middleTarget.addVectors(armPosition, belowVector);
      }

      armObject3D.lookAt(middleTarget);
      armObject3D.rotateX(Math.PI / 2);
      foreArmObject3D.lookAt(controllerPosition);
      foreArmObject3D.rotateX(Math.PI / 2);

      // handObject3D.rotation = controllerObject3D.rotation;
      // foreArmObject3D.rotateOnAxis(foreArmObject3D.up, -controllerObject3D.rotation.z - Math.PI / 2);
      // TODO modify hand rotation
      // handObject3D.rotation.x = controllerObject3D.rotation.y;
      // handObject3D.rotation.y = controllerObject3D.rotation.z - Math.PI / 2;
      // handObject3D.rotation.z = controllerObject3D.rotation.x - Math.PI / 2;
    } else {
      const distance = armPosition.distanceTo(controllerPosition);
      let middleTarget = new THREE.Vector3();
      if (distance >= this.rightArmLength) {
        middleTarget = controllerPosition;
      } else {
        const forward = new THREE.Vector3();
        controllerObject3D.getWorldDirection(forward);
        forward.normalize();
        forward.multiplyScalar(-this.rightHandToForeArm);
        middleTarget.subVectors(controllerPosition, forward);
      }

      armObject3D.lookAt(middleTarget);
      armObject3D.rotateX(Math.PI / 2);
      foreArmObject3D.lookAt(controllerPosition);
      foreArmObject3D.rotateX(Math.PI / 2);
    }
    armObject3D.matrixNeedsUpdate = true;
    foreArmObject3D.matrixNeedsUpdate = true;
    handObject3D.matrixNeedsUpdate = true;
  },
  updateArm(
    handRotation,
    handObject3D,
    controllerObject3D,
    isLeft,
    isInView,
    shoulderObject3D,
    armObject3D,
    foreArmObject3D
  ) {
    const handPosition = new THREE.Vector3();
    handObject3D.getWorldPosition(handPosition);
    const foreArmPosition = new THREE.Vector3();
    foreArmObject3D.getWorldPosition(foreArmPosition);
    const armPosition = new THREE.Vector3();
    armObject3D.getWorldPosition(armPosition);
    const shoulderPosition = new THREE.Vector3();
    shoulderObject3D.getWorldPosition(shoulderPosition);
    const controllerPosition = new THREE.Vector3();
    controllerObject3D.getWorldPosition(controllerPosition);
    const controllerRotation = controllerObject3D.rotation;

    const elbowPosition = this.calcElbowPosition(
      isLeft,
      armPosition,
      controllerPosition,
      controllerRotation,
      this.leftForeArmToArm,
      this.leftHandToForeArm
    );
    armObject3D.lookAt(elbowPosition);
    armObject3D.rotateX(Math.PI / 2);
    foreArmObject3D.lookAt(controllerPosition);
    foreArmObject3D.rotateX(Math.PI / 2);
    if (isLeft) {
      foreArmObject3D.rotateOnAxis(foreArmObject3D.up, -controllerObject3D.rotation.z - Math.PI / 2);
    } else {
      foreArmObject3D.rotateOnAxis(foreArmObject3D.up, -controllerObject3D.rotation.z + Math.PI / 2);
    }
    this.fixForeArmRotation(isLeft, foreArmObject3D, controllerPosition, armPosition);
    // handObject3D.rotation.x = -controllerObject3D.rotation.x + foreArmObject3D.rotation.x;
    // handObject3D.rotation.z = -controllerObject3D.rotation.y - foreArmObject3D.rotation.z;
    const handMatrix = handObject3D.matrix;
    // handMatrix.multiplyMatrices(this.invRootToChest, controllerObject3D.matrix);
    const rootToForeArm = new THREE.Matrix4();
    const invRootToForeArm = new THREE.Matrix4();
    rootToForeArm.multiplyMatrices(this.rootToChest, shoulderObject3D.matrix);
    rootToForeArm.multiply(armObject3D.matrix);
    rootToForeArm.multiply(foreArmObject3D.matrix);
    invRootToForeArm.getInverse(rootToForeArm);
    handMatrix.multiplyMatrices(invRootToForeArm, controllerObject3D.matrix);

    handMatrix.multiply(handRotation);

    // handObject3D.position.setFromMatrixPosition(handMatrix);
    handObject3D.rotation.setFromRotationMatrix(handMatrix);

    armObject3D.matrixNeedsUpdate = true;
    foreArmObject3D.matrixNeedsUpdate = true;
    handObject3D.matrixNeedsUpdate = true;
  },
  fixForeArmRotation(isLeft, foreArmObject3D, handPosition, shoulderPosition) {
    // reduces calculation problems when hand is moving around shoulder XZ coordinates -> forces elbow to be outside of body
    const handRelativePosition = new THREE.Vector3();
    handRelativePosition.subVectors(handPosition, shoulderPosition);
    const startBelowDistance = 0.5;
    const startBelowY = 0.1;
    const sWeight = 2;
    const localElbowPos = new THREE.Vector3(0.3, -1, -2);
    const shoulderHandDirection = new THREE.Vector3();
    shoulderHandDirection.subVectors(handPosition, shoulderPosition).normalize();
    if (isLeft) {
      localElbowPos.x *= -1;
    }
    const distanceZ = Math.abs(handRelativePosition.x);
    const distanceY = Math.abs(handRelativePosition.y);
    const distanceXZ = Math.sqrt(handRelativePosition.z ** 2 + handRelativePosition.x ** 2);
    const upperAxisW = Math.max(0, Math.min(1, (startBelowDistance - distanceXZ) / startBelowDistance));
    const backW = Math.max(0, Math.min(1, (0.1 - distanceZ) * 3));
    const upW = Math.max(0, Math.min(1, (startBelowY - distanceY) / startBelowY));

    const weight = Math.max(0, Math.min(1, upperAxisW * sWeight + backW)) * upW;
    const delta = weight * Math.PI * 2;
    foreArmObject3D.rotateOnAxis(foreArmObject3D.up, delta);
  },
  calcElbowPosition(isLeft, shoulderPosition, handPosition, handRotation, shoulderToElbow, handToElbow) {
    const armLength = shoulderToElbow + handToElbow;
    const lu = shoulderToElbow;
    const ll = handToElbow;
    const Xs = shoulderPosition;
    const Xh = handPosition;

    const handRelativePosition = new THREE.Vector3();
    handRelativePosition.subVectors(Xh, Xs);
    const Xsh = [
      -handRelativePosition.z / armLength,
      handRelativePosition.y / armLength,
      -handRelativePosition.x / armLength
    ];
    const offsetAngle = 135;
    const xDistanceStart = 0.1;
    const zDistanceStart = 0.6;
    const w = [-50, -60, 260, -100];
    let p = offsetAngle;
    // angle from Y
    p += Xsh[1] * w[1];
    // angle from Z
    if (Xsh[1] > 0) {
      p += w[2] * Math.max(zDistanceStart - Xsh[2], 0) * Math.max(Xsh[1], 0);
    } else {
      p += w[3] * Math.max(zDistanceStart - Xsh[2], 0) * Math.max(-Xsh[1], 0);
    }
    // angle from X
    p += w[0] * Math.max(Xsh[0] * (isLeft ? 1 : -1) + xDistanceStart, 0);
    let clampedP = Math.min(Math.max(13, p), 175);
    if (!isLeft) {
      clampedP *= -1;
    }
    // const radP = -(Math.PI * clampedP) / 180 + Math.PI / 2;

    const radP = -(Math.PI * clampedP) / 180 + Math.PI / 2;

    // TODO Elbow rotation from wrist rotation
    // const handDeltaPow = 1.5;
    // const handDeltaFactor = -0.3;
    // const handDeltaOffset = 45;
    // const rotateElbowWithHandDelay = 0.08;

    // const yawLowThreshold = -45;
    // const yawLowConstant = -1 / 135;
    // const yawUpperThreshold = 135;
    // const yawUpperConstant = 1 / 135;
    // const rollLowThreshold = 0;
    // const rollLowConstant = -1 / 600;
    // const rollUpperThreshold = 90;
    // const rollUpperConstant = 1 / 300;

    const vsh = new THREE.Vector3();
    vsh.subVectors(handPosition, shoulderPosition);
    const yn = vsh.clone();
    yn.normalize();
    const n = yn.clone();
    const y = new THREE.Vector3(0, 1, 0);
    yn.multiplyScalar(y.dot(yn));
    vsh.multiplyScalar(lu ** 2 / (lu ** 2 + ll ** 2));

    const shoulderToHand = shoulderPosition.distanceTo(handPosition);
    const s = (lu + ll + shoulderToHand) / 2;
    const area = Math.sqrt(s * (s - lu) * (s - ll) * (s - shoulderToHand));
    const r = isNaN((2 * area) / shoulderToHand) ? 0 : (2 * area) / shoulderToHand;

    const u = new THREE.Vector3();
    u.subVectors(y, yn);
    u.normalize();
    const v = new THREE.Vector3();
    v.crossVectors(u, n);
    const centerToElbow = new THREE.Vector3();
    v.multiplyScalar(r * Math.cos(radP));
    u.multiplyScalar(r * Math.sin(radP));
    centerToElbow.addVectors(v, u);

    const elbowPosition = new THREE.Vector3();
    elbowPosition.addVectors(shoulderPosition, vsh);
    elbowPosition.add(centerToElbow);

    return elbowPosition;
  },
  // vrdevel modified end

  _runScheduledWork() {
    // Every scheduled run, we force an IK update on the next frame (so at most one avatar with forced IK per frame)
    // and also update the this.isInView bit on the avatar which is used to determine if an IK update should be run
    // every frame.
    this.forceIkUpdate = true;

    this._updateIsInView();
  },

  _updateIsInView: (function() {
    const frustum = new THREE.Frustum();
    const frustumMatrix = new THREE.Matrix4();
    const cameraWorld = new THREE.Vector3();
    const isInViewOfCamera = (screenCamera, pos) => {
      frustumMatrix.multiplyMatrices(screenCamera.projectionMatrix, screenCamera.matrixWorldInverse);
      frustum.setFromMatrix(frustumMatrix);
      return frustum.containsPoint(pos);
    };

    return function() {
      if (!this.playerCamera) return;

      const camera = this.ikRoot.camera.object3D;
      camera.getWorldPosition(cameraWorld);

      // Check player camera
      this.isInView = isInViewOfCamera(this.playerCamera, cameraWorld);

      if (!this.isInView) {
        // Check in-game camera if rendering to viewfinder and owned
        const cameraTools = this.el.sceneEl.systems["camera-tools"];

        if (cameraTools) {
          cameraTools.ifMyCameraRenderingViewfinder(cameraTool => {
            this.isInView = this.isInView || isInViewOfCamera(cameraTool.camera, cameraWorld);
          });
        }
      }
    };
  })()
});
