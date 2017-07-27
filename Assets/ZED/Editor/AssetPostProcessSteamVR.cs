using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
public class AssetPostProcessSteamVR : AssetPostprocessor
{

    static void OnPostprocessAllAssets(string[] importedAssets, string[] deletedAssets, string[] movedAssets, string[] movedFromAssetPaths)
    {

        if (ZEDDependenciesUpdater.CheckPackageExists("SteamVR"))
        {
            ZEDDependenciesUpdater.ActivateDefine();
        } else
        {
            ZEDDependenciesUpdater.DesactivateDefine();
        }
    }
}
