using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

/// <summary>
/// Utility class to do save/load files into different formats.
/// </summary>
public static class FileUtility {

    /// <summary>
    /// Saves a texture to the assets folder.
    /// </summary>
    /// <param name="tex">The texture you want to store.</param>
    public static void SaveTextureToFile(Texture2D tex, string filename)
    {
        byte[] bytes = tex.EncodeToPNG();
        var filestream = File.Open(Application.dataPath + "/" + filename, FileMode.OpenOrCreate, FileAccess.ReadWrite);
        var binarywriter = new BinaryWriter(filestream);
        binarywriter.Write(bytes);
        filestream.Close();
    }

    /// <summary>
    /// Loads a texture from the assets folder.
    /// </summary>
    /// <param name="filename">The filename of the texture in the assets folder.</param>
    private static Texture2D LoadTextureFromFile(string filename)
    {
        Texture2D tex = null;
        byte[] fileData;
        string filePath = Application.dataPath + "/" + filename;

        fileData = File.ReadAllBytes(filePath);
        tex = new Texture2D(2, 2);
        tex.LoadImage(fileData); //..this will auto-resize the texture dimensions.
        return tex;
    }
}
