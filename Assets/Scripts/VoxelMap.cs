using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using UnityEngine;

/*
byte legend:
first 2 bits
00 = unknown
01 = occupied
10 = free

last bit stores clearance
*/

public class VoxelMap
{
    Vector3 mapOrigin;
    int chunkSize;
    float voxelSize;
    Dictionary<Vector3Int, byte[]> chunkMap;
    List<Matrix4x4> renderMatrices;
    Dictionary<Vector3Int, int> indexMap;

    public VoxelMap(Vector3 mapOrigin, int chunkSize, float voxelSize)
    {
        this.mapOrigin = mapOrigin;
        this.chunkSize = chunkSize;
        this.voxelSize = voxelSize;
        chunkMap = new();
        renderMatrices = new();
        indexMap = new();
    }


    public void Add(Vector3Int voxel, byte state)
    {
        Vector3Int localVoxel = GetLocalVoxel(voxel);
        Vector3Int chunkOrigin = voxel - localVoxel;

        byte[] chunk = chunkMap.TryGetValue(chunkOrigin, out var expected) ? expected : (chunkMap[chunkOrigin] = new byte[chunkSize * chunkSize * chunkSize]);
        int index = GetIndex(localVoxel);

        if ((state == 1 || (chunk[index] & 0b11) == 1) && state != (chunk[index] & 0b11))
        {
            if (state != 1)
            {
                int lastIndex = renderMatrices.Count - 1;
                int removingIndex = indexMap[voxel];
                if (lastIndex == removingIndex)
                {
                    renderMatrices.RemoveAt(lastIndex);
                }
                else
                {
                    Matrix4x4 switching = renderMatrices[lastIndex];
                    renderMatrices[removingIndex] = switching;
                    Vector3Int key = Vector3Int.FloorToInt((switching.GetPosition() - mapOrigin) / voxelSize - Vector3.one * 0.5f);
                    indexMap[key] = removingIndex;
                    renderMatrices.RemoveAt(lastIndex);
                }
                indexMap.Remove(voxel);
            }
            else
            {
                indexMap[voxel] = renderMatrices.Count;
                Vector3 worldPos = mapOrigin + ((Vector3)voxel + Vector3.one * 0.5f) * voxelSize;
                renderMatrices.Add(Matrix4x4.TRS(worldPos, Quaternion.identity, Vector3.one * voxelSize));
            }
        }

        chunk[index] = (byte)((chunk[index] & 0b11111100) | state);
    }

    public byte Get(Vector3Int voxel)
    {
        Vector3Int localVoxel = GetLocalVoxel(voxel);
        Vector3Int chunkOrigin = voxel - localVoxel;

        if (chunkMap.TryGetValue(chunkOrigin, out var chunk))
            return (byte)(chunk[GetIndex(localVoxel)] & 0b11);

        return 0;
    }

    public void SetClearance(Vector3Int voxel, bool clearance)
    {
        Vector3Int localVoxel = GetLocalVoxel(voxel);
        Vector3Int chunkOrigin = voxel - localVoxel;

        byte[] chunk = chunkMap.TryGetValue(chunkOrigin, out var expected) ? expected : (chunkMap[chunkOrigin] = new byte[chunkSize * chunkSize * chunkSize]);
        int index = GetIndex(localVoxel);

        chunk[index] = (byte)((chunk[index] & 0b11) | ((clearance ? 0 : 1) << 2));
    }

    public bool CanTraverse(Vector3Int voxel)
    {
        Vector3Int localVoxel = GetLocalVoxel(voxel);
        Vector3Int chunkOrigin = voxel - localVoxel;

        if (chunkMap.TryGetValue(chunkOrigin, out var chunk))
        {
            int voxelInfo = chunk[GetIndex(localVoxel)];
            return (voxelInfo & (1 << 2)) == 0 && (voxelInfo & 0b11) == 2;
        }

        return false;
    }

    private Vector3Int GetLocalVoxel(Vector3Int voxel)
    {
        int x = voxel.x % chunkSize;
        int y = voxel.y % chunkSize;
        int z = voxel.z % chunkSize;

        if (x < 0) x += chunkSize;
        if (y < 0) y += chunkSize;
        if (z < 0) z += chunkSize;

        return new Vector3Int(x, y, z);
    }

    public int GetIndex(Vector3Int localVoxel)
    {
        return localVoxel.x + (localVoxel.y * chunkSize) + (localVoxel.z * chunkSize * chunkSize);
    }

    public Vector3Int IndexToLocalVoxel(int index)
    {
        int z = index / (chunkSize * chunkSize);
        int y = (index / chunkSize) % chunkSize;
        int x = index % chunkSize;

        return new Vector3Int(x, y, z);
    }

    public void Render(Mesh mesh, Material mat)
    {
        for (int i = 0; i < renderMatrices.Count; i += 1023)
        {
            int batchSize = Mathf.Min(1023, renderMatrices.Count - i);
            Graphics.DrawMeshInstanced(mesh, 0, mat, renderMatrices.GetRange(i, batchSize));
        }
    }
}