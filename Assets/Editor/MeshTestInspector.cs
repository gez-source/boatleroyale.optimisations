using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(MeshRenderer))]
public class MeshTestInspector : Editor
{
	public int tooManyLabelsThreshold = 50;
	int maxVertsToLabel = 0;

	private void OnSceneGUI()
	{
		MeshFilter meshFilter = target as MeshFilter;


		if (meshFilter != null)
		{
			Mesh mesh = meshFilter.mesh;

			maxVertsToLabel = mesh.vertices.Length;

			if (maxVertsToLabel > tooManyLabelsThreshold)
			{
				maxVertsToLabel = tooManyLabelsThreshold;
			}

			Transform transform = ((MeshFilter)target).transform;
			for (var index = 0; index < maxVertsToLabel; index++)
			{
				Vector3 vertex = mesh.vertices[index] + new Vector3();

				Handles.ArrowHandleCap(0, mesh.vertices[index], Quaternion.Euler(mesh.normals[index]), 10f, EventType.Ignore);
				Handles.ArrowHandleCap(0, Vector3.zero, Quaternion.LookRotation(Vector3.right), 1000f, EventType.Ignore);

				Handles.Label(transform.position + vertex, index.ToString());
				Handles.Label(transform.position + transform.TransformDirection(vertex), vertex.y.ToString());

				if (Event.current.type == EventType.Repaint)
				{
					Handles.color = Handles.xAxisColor;
					float size = 1f;
					Handles.ArrowHandleCap(
						0,
						transform.position + transform.TransformDirection(mesh.vertices[index]),
						transform.rotation * Quaternion.LookRotation(mesh.normals[index]),
						size,
						EventType.Repaint
					);
				}
			}
		}


		//
		//		foreach (int triangle in meshFilter.triangles)
		//		{
		//			print(triangle);
		//		}
		//
		//		foreach (Vector3 normal in meshFilter.normals)
		//		{
		//			print(normal);
		//		}
	}


	// Update is called once per frame
}
