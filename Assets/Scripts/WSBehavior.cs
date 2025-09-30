using UnityEngine;
using System;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

public class WSBehaviour : MonoBehaviour
{
    private ClientWebSocket ws;
    private SynchronizationContext mainThread;

    async void Start()
    {
        mainThread = SynchronizationContext.Current;

        var main = GetComponent<Main>();
        ws = new ClientWebSocket();
        var uri = new Uri($"wss://api.fleetcontrol.coskyler.com/ws/unity?voxelSize={main.voxelSize}&startTime=1695142600&drone=Alpha");

        await ws.ConnectAsync(uri, CancellationToken.None);
        Debug.Log("Connected");

        _ = Task.Run(ReceiveLoop);
    }

    public async Task Send(string msg)
    {
        if (ws?.State == WebSocketState.Open)
        {
            var bytes = Encoding.UTF8.GetBytes(msg);
            await ws.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, CancellationToken.None);
        }
    }

    private async Task ReceiveLoop()
    {
        var buf = new byte[4096];
        while (ws.State == WebSocketState.Open)
        {
            var sb = new StringBuilder();
            WebSocketReceiveResult result;
            do
            {
                result = await ws.ReceiveAsync(new ArraySegment<byte>(buf), CancellationToken.None);
                if (result.MessageType == WebSocketMessageType.Close)
                {
                    await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Closing", CancellationToken.None);
                    Debug.Log("Closed by server.");
                    return;
                }
                sb.Append(Encoding.UTF8.GetString(buf, 0, result.Count));
            } while (!result.EndOfMessage);

            var msg = sb.ToString().Trim();
            Debug.Log($"Received: '{msg}'");

            if (msg.Equals("start", StringComparison.OrdinalIgnoreCase))
            {
                mainThread.Post(_ =>
                {
                    var fc = GetComponent<FlightController>();
                    if (fc != null) fc.maxSpeed = 2.5f;

                    var main = GetComponent<Main>();
                    if (main != null) main.scanStarted = true;

                    Debug.Log("STARTING!!!");
                }, null);
            }
        }
    }

    private async void OnApplicationQuit()
    {
        if (ws != null && ws.State == WebSocketState.Open)
            await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Quit", CancellationToken.None);
    }
}
