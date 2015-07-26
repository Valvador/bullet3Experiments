#include <DirectXMath.h>
#include <assert.h>
using namespace DirectX;

#ifndef RIGIDBODY_C_H
#define RIGIDBODY_C_H

public class RigidBody_c // DirectX Implementation
{
public:
	//<---------LINEAR----------------><-------------ANGULAR-------------->
	XMFLOAT3 m_position;				XMFLOAT4 m_orientation;
	XMFLOAT3 m_linearVelocity;			XMFLOAT3 m_angularVelocity;
	XMFLOAT3 m_force;					XMFLOAT3 m_torque;
	float m_invMass;					XMFLOAT3X3 m_invInertia;
	void AddForce(const XMFLOAT3& worldPosForce,
				  const XMFLOAT3& directionMagnitude)
	{
		//<---------LINEAR----------------->
		XMVECTOR newForce = XMLoadFloat3(&m_force) + XMLoadFloat3(&directionMagnitude);
		XMStoreFloat3(&m_force, newForce);

		//<-----------ANGULAR-------------->
		XMVECTOR distance = (XMLoadFloat3(&worldPosForce) - XMLoadFloat3(&m_position));
		XMFLOAT3 torque;
		XMStoreFloat3(&torque, XMVector3Cross(distance, XMLoadFloat3(&directionMagnitude)));
		AddTorque(torque);
	}

	void AddTorque(XMFLOAT3 worldAxisAndMagnitudeTorque)
	{
		XMVECTOR newTorque = XMLoadFloat3(&m_torque) + XMLoadFloat3(&worldAxisAndMagnitudeTorque);
		XMStoreFloat3(&m_torque, newTorque);
	}

	XMFLOAT3X3 CreateWorldII() const
	{
		XMMATRIX orientationMatrix			= XMMatrixRotationQuaternion(XMLoadFloat4(&m_orientation));
		XMMATRIX inverseOrientationMatrix	= XMMatrixTranspose(orientationMatrix);
		XMMATRIX inverseWorldInertiaMatrix = inverseOrientationMatrix * XMLoadFloat3x3(&m_invInertia) *
			orientationMatrix;

		XMFLOAT3X3 output;
		XMStoreFloat3x3(&output, inverseWorldInertiaMatrix);
		return output;
	}
};

#endif