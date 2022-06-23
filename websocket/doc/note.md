Websocket là một chuẩn giao tiếp máy tính, hỗ trợ các channel giao tiếp full-duplex qua một kết nối TCP.
Về bản chất websocket khác với HTTP mặc dù cả 2 giao thức đều trên layer 7 của mô hình OSI và cùng phụ thuộc vào TCP ở layer 4. Tuy nhiên websocket được thiết kế để hoạt động trên các cổng HTTP 443 và 80 => hoàn toàn có khả năng tương thích với giao thức HTTP.
=> Để có được sự tương thích này, websocket handshake sẽ sử dụng một header HTTP Upgrade để thay đổi giao thức HTTP thành websocket.
Websocket hỗ trợ tương tác giữa một trình duyệt web (hay ứng dụng client) với một web server nhưng chi phí thấp hơn so với half-duplex HTTP polling => việc truyền dữ liệu theo thời gian thực với server sẽ trở nên hiệu quả hơn.
Websocket sẽ cung cấp một cách để server có thể gửi content đến client mà không cần được request bởi client. Đồng thời cho phép các thông báo được truyền qua lại trong khi vẫn giữ cho kết nối được mở. Từ đó tạo nên một giao tiếp hai chiều giữa client và server.

Thiết lập kết nối websocket: URI websocket sẽ sử dụng giao thức "ws:" (hay "wss:" với websocket bảo mật). Phần còn lại của URI sẽ tương tự với HTTP URI: gồm host, port, path ...
    "ws:" "//" host [ ":" port] part [ "?" query ]
    "wss:" "//" host [ ":" port] part [ "?" query ]
Kết nối websocket được thiếp lập bằng cách upgrading một cặp HTTP request/response.