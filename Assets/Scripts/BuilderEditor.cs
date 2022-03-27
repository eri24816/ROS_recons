using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(Builder))]
public class BuilderEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        Builder builder = (Builder)target;
        if (GUILayout.Button("Built pixel"))
        {
            builder.BuildPixel((float[,])builder.map.Clone());
        }
        if (GUILayout.Button("Build walk around"))
        {
            builder.BuildWalkAround((float[,])builder.map.Clone());
        }
        if (GUILayout.Button("Generate map"))
        {
            builder.GenerateMap();
        }
        if (GUILayout.Button("Read map"))
        {
            builder.ReadMap();
        }
    }
}
